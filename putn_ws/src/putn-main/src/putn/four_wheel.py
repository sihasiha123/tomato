#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import can
import struct
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class FixedDistanceController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fixed_distance_controller', anonymous=True)
        
        # 车辆参数
        self.wheelbase = rospy.get_param('~wheelbase', 0.5)  # 轴距 (m)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', np.pi/4)  # 最大转角 (rad)
        
        # 运动参数
        self.target_distance = 5.0  # 目标距离 (m)
        self.linear_speed = 0.5     # 线速度 (m/s)
        self.move_duration = self.target_distance / self.linear_speed  # 运动时间 (s)
        
        # 障碍物检测参数
        self.obstacle_detected = False      # 障碍物检测标志位
        self.obstacle_distance = 2.0        # 障碍物检测距离 (m)
        self.front_angle_range = 30.0       # 前方检测角度范围 (度)
        self.min_height = -0.5              # 最小检测高度 (m)
        self.max_height = 2.0               # 最大检测高度 (m)
        self.min_points_threshold = 10      # 最小点数阈值
        
        # 运动状态
        self.current_state = "IDLE"         # 当前状态: IDLE, FORWARD, BACKWARD, STOPPED
        self.distance_traveled = 0.0        # 已行驶距离
        self.move_start_time = 0.0          # 运动开始时间
        
        # 初始化CAN总线
        try:
            self.can_bus = can.interface.Bus(channel='can0', 
                                           bustype='socketcan',
                                           bitrate=1000000)
            rospy.loginfo("CAN总线初始化成功")
        except Exception as e:
            rospy.logerr(f"CAN总线初始化失败: {e}")
            return
        
        # 订阅点云话题
        self.pointcloud_sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.pointcloud_callback)
        
        rospy.loginfo(f"固定距离控制器启动")
        rospy.loginfo(f"目标距离: {self.target_distance}m, 速度: {self.linear_speed}m/s")
        rospy.loginfo(f"障碍物检测距离: {self.obstacle_distance}m, 前方角度范围: ±{self.front_angle_range/2}°")
        
    def pointcloud_callback(self, cloud_msg):
        """
        处理点云数据，只检测前方障碍物
        """
        try:
            # 重置障碍物检测标志
            obstacle_points = 0
            
            # 转换点云数据
            points = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            
            for point in points:
                x, y, z = point
                
                # 只检测前方区域 (x > 0，前方为正x方向)
                if x <= 0:
                    continue
                
                # 计算到车辆的距离
                distance = np.sqrt(x*x + y*y)
                
                # 距离筛选：只检测指定范围内的点
                if distance > self.obstacle_distance:
                    continue
                
                # 高度筛选：排除地面和过高的点
                if z < self.min_height or z > self.max_height:
                    continue
                
                # 角度筛选：只检测前方指定角度范围内的点
                angle_deg = np.rad2deg(np.arctan2(abs(y), x))
                if angle_deg > self.front_angle_range / 2:
                    continue
                
                # 符合条件的障碍物点
                obstacle_points += 1
                
                # 如果检测到足够多的点，认为有障碍物
                if obstacle_points >= self.min_points_threshold:
                    break
            
            # 更新障碍物检测标志位
            previous_state = self.obstacle_detected
            self.obstacle_detected = obstacle_points >= self.min_points_threshold
            
            # 只在状态变化时打印日志
            if previous_state != self.obstacle_detected:
                if self.obstacle_detected:
                    rospy.logwarn(f"检测到前方障碍物！检测点数: {obstacle_points}")
                else:
                    rospy.loginfo("前方障碍物清除")
            
        except Exception as e:
            rospy.logerr(f"点云处理失败: {e}")
    
    def send_can_message(self, linear_x, steering_angle=0.0):
        """
        发送CAN控制消息
        """
        try:
            # 限制转角范围
            steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
            
            # 转换单位
            # 线速度: m/s -> mm/s (乘以1000)
            linear_x_mms = int(linear_x * 1000)
            # 转角: rad -> 0.01rad (乘以100)
            steering_angle_crad = int(steering_angle * 100)
            
            # 限制数值范围 (16位有符号整数)
            linear_x_mms = max(-32768, min(32767, linear_x_mms))
            steering_angle_crad = max(-32768, min(32767, steering_angle_crad))
            
            # 构造CAN数据帧
            can_id = 0x00A
            data = bytearray(8)
            
            # 字节0: 指令字节 
            data[0] = 0x0C
            
            # 转角处理 (转换为16位有符号整数)
            if steering_angle_crad >= 0:
                steering_data = steering_angle_crad & 0xFFFF
            else:
                # 负数使用二补数表示
                steering_data = (0x10000 + steering_angle_crad) & 0xFFFF
            
            # 字节1-2: 转角 (大端格式：高字节在前)
            data[1] = (steering_data >> 8) & 0xFF    # 高字节
            data[2] = steering_data & 0xFF           # 低字节
            
            # 线速度处理 (转换为16位有符号整数)
            if linear_x_mms >= 0:
                linear_data = linear_x_mms & 0xFFFF
            else:
                # 负数使用二补数表示
                linear_data = (0x10000 + linear_x_mms) & 0xFFFF
            
            # 字节3-4: 线速度 (大端格式：高字节在前)
            data[3] = (linear_data >> 8) & 0xFF     # 高字节
            data[4] = linear_data & 0xFF            # 低字节
            
            # 字节5-7: 保留字段 (填充0)
            data[5] = 0x00
            data[6] = 0x00
            data[7] = 0x00
            
            # 创建CAN消息
            can_msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            
            # 发送CAN消息
            self.can_bus.send(can_msg)
            
            # 只在状态变化时打印详细信息
            if linear_x != 0:
                data_hex = ' '.join([f'{b:02X}' for b in data])
                rospy.loginfo(f"发送CAN帧: ID=0x{can_id:03X}, Data={data_hex}")
                rospy.loginfo(f"线速度: {linear_x:.3f} m/s ({linear_x_mms} mm/s), "
                             f"转角: {np.rad2deg(steering_angle):.2f}° ({steering_angle_crad} crad)")
            
        except Exception as e:
            rospy.logerr(f"发送CAN消息失败: {e}")
    
    def stop_vehicle(self):
        """
        停止车辆
        """
        rospy.loginfo("停止车辆")
        self.send_can_message(0.0, 0.0)
        self.current_state = "STOPPED"
        
    def move_forward(self, distance):
        """
        前进指定距离（带障碍物检测，遇到障碍物时停止等待）
        """
        rospy.loginfo(f"开始前进 {distance}m")
        self.current_state = "FORWARD"
        self.move_start_time = time.time()
        self.distance_traveled = 0.0
        
        # 计算目标时间
        target_duration = distance / self.linear_speed
        
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - self.move_start_time
            self.distance_traveled = elapsed_time * self.linear_speed
            
            # 检查是否到达目标距离（修改：无论是否有障碍物都执行回退）
            if elapsed_time >= target_duration:
                rospy.loginfo(f"到达目标位置 ({distance}m)，准备执行回退操作")
                self.stop_vehicle()
                time.sleep(1.0)  # 短暂停止
                return True  # 返回True表示到达目标，需要回退
            
            # 检查障碍物（遇到障碍物时停止等待，但不回退）
            if self.obstacle_detected:
                rospy.logwarn("检测到前方障碍物，停止前进等待障碍物清除")
                self.stop_vehicle()
                
                # 等待障碍物清除
                while self.obstacle_detected and not rospy.is_shutdown():
                    rospy.loginfo("等待障碍物清除...")
                    time.sleep(0.5)
                
                if not rospy.is_shutdown():
                    rospy.loginfo("障碍物已清除，继续前进")
                    self.move_start_time = time.time() - elapsed_time  # 调整开始时间
                    continue
            
            # 发送前进指令
            self.send_can_message(self.linear_speed, 0.0)
            time.sleep(0.1)  # 100ms发送一次
        
        # 如果程序被中断，停止车辆
        self.stop_vehicle()
        return False
        
    def move_backward(self, distance):
        """
        后退指定距离
        """
        rospy.loginfo(f"开始后退 {distance}m")
        self.current_state = "BACKWARD"
        
        # 后退时不检测障碍物，直接按时间执行
        start_time = time.time()
        duration = distance / self.linear_speed
        
        while time.time() - start_time < duration and not rospy.is_shutdown():
            # 发送后退指令
            self.send_can_message(-self.linear_speed, 0.0)
            time.sleep(0.1)  # 100ms发送一次
            
        # 停止
        self.stop_vehicle()
        rospy.loginfo(f"后退 {distance}m 完成")
    
    def execute_fixed_pattern(self):
        """
        执行固定运动模式：前进5m -> 停2s -> 后退5m -> 停2s
        修改逻辑：只要到达目标点就执行回退操作，无论是否检测到障碍物
        """
        try:
            rospy.loginfo("="*50)
            rospy.loginfo("开始执行固定运动模式")
            rospy.loginfo("逻辑：到达目标点即执行回退，无论是否有障碍物")
            rospy.loginfo("="*50)
            
            # 初始停止确保车辆静止
            self.stop_vehicle()
            time.sleep(1.0)
            
            # 1. 前进5米（到达目标点就返回True）
            reached_target = self.move_forward(5.0)
            
            if reached_target:
                rospy.loginfo("已到达目标点，执行回退操作")
            else:
                rospy.logwarn("前进过程被中断，仍将执行回退操作")
            
            # 2. 停止2秒
            rospy.loginfo("停止2秒...")
            time.sleep(2.0)
            
            # 3. 后退5米（无论如何都执行）
            self.move_backward(5.0)
            
            # 4. 最终停止2秒
            rospy.loginfo("停止2秒...")
            time.sleep(2.0)
            
            rospy.loginfo("="*50)
            rospy.loginfo("固定运动模式执行完成")
            rospy.loginfo("="*50)
            
        except Exception as e:
            rospy.logerr(f"执行运动模式时出错: {e}")
            self.stop_vehicle()
    
    def run(self):
        """
        运行控制器
        """
        try:
            # 等待一下确保系统就绪
            rospy.loginfo("等待点云数据...")
            time.sleep(3.0)
            
            # 执行固定运动模式
            self.execute_fixed_pattern()
            
            # 保持节点运行，可以重复执行
            rospy.loginfo("程序执行完成，按 Ctrl+C 退出程序")
            rospy.spin()
            
        except rospy.ROSInterruptException:
            rospy.loginfo("程序被用户中断")
        except Exception as e:
            rospy.logerr(f"程序运行错误: {e}")
        finally:
            # 确保最终停止
            self.stop_vehicle()
        
    def __del__(self):
        """
        清理资源
        """
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()

if __name__ == '__main__':
    try:
        controller = FixedDistanceController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")