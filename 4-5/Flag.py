#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上轨道往返状态机

流程:
1. 启动后默认切到等待模式(默认横移模式 0x04)
2. 等待 ROS 标志位触发
3. 收到标志位后切到行驶模式(默认 0x02)
4. 记录轮式里程计起点(接收四轮 0x25 上报)
5. 大速度直行上轨
6. 停车后倒车回零
7. 若后面还有下一轮,切回横移模式并执行横移位置控制
8. 若当前已是最后一轮,回零后直接结束
9. 通过完成信号话题上报本轮结果,继续等待下一次标志位

说明:
- 速度命令按用户要求使用 0x0C 00 00 [线速度hi lo] [角速度hi lo] 00
- 线速度单位先按 mm/s
- 角速度当前默认给 0
- 轮式里程计从 CAN 异步上报中接收:
  ID in {1,2,3,4} 且 data[0] == 0x25
"""

import sys
import time
import signal
import threading
from collections import deque

import rospy
from std_msgs.msg import Int32

try:
    import can
except ImportError:
    can = None


# 与 demo/yes_3_fix_3.py 中的 UnitConverter.HALL_TO_MM 保持一致
DEFAULT_WHEEL_HALL_TO_MM = 2.38
class UpRailFlagController:
    def __init__(self):
        rospy.init_node("yes_3_fix_3_uprail_flag", anonymous=True)

        # ========== CAN / 模式 ==========
        self.can_id = int(rospy.get_param("~can_id", 0x00A))
        self.can_channel = rospy.get_param("~can_channel", "can0")
        self.can_bitrate = int(rospy.get_param("~can_bitrate", 1000000))
        wheel_odom_ids = rospy.get_param("~wheel_odom_ids", [1, 2, 3, 4])
        self.wheel_odom_ids = tuple(sorted({int(x) for x in wheel_odom_ids}))
        self.wheel_odom_id_set = set(self.wheel_odom_ids)
        self.wheel_hall_to_mm = float(rospy.get_param("~wheel_hall_to_mm", DEFAULT_WHEEL_HALL_TO_MM))
        self.wheel_odom_timeout_sec = float(rospy.get_param("~wheel_odom_timeout_sec", 0.5))
        self.wheel_odom_use_total_hall = bool(rospy.get_param("~wheel_odom_use_total_hall", False))
        self.MODE_DRIVE = int(rospy.get_param("~drive_mode_code", 0x02))
        self.MODE_LATERAL = int(rospy.get_param("~wait_mode_code", 0x04))
        self.MODE_SWITCH_QUERY_INTERVAL = float(rospy.get_param("~mode_switch_query_interval", 0.15))
        self.MODE_SWITCH_WAIT_TIME = float(rospy.get_param("~mode_switch_wait_time", 3.0))
        self.MOTION_QUERY_INTERVAL = float(rospy.get_param("~motion_query_interval", 0.25))
        self.QUERY_TIMEOUT = float(rospy.get_param("~query_timeout", 10.0))

        # ========== 触发标志位 ==========
        self.flag_topic = rospy.get_param("~flag_topic", "/flag1")
        self.trigger_value = int(rospy.get_param("~trigger_value", 1))
        self.done_topic = rospy.get_param("~done_topic", "/rail_cycle_done")
        self.workflow_param_ns = rospy.get_param("~workflow_param_ns", "/rail_workflow")
        self.planned_cycle_count = int(
            rospy.get_param(
                "~planned_cycle_count",
                rospy.get_param(f"{self.workflow_param_ns}/cycle_count", 2),
            )
        )

        # ========== 运动参数 ==========
        self.forward_linear_speed_mm_s = int(rospy.get_param("~forward_linear_speed_mm_s", 1400))
        self.forward_distance_mm = float(rospy.get_param("~forward_distance_mm", 2500.0))
        self.forward_distance_hall = float(rospy.get_param("~forward_distance_hall", 0.0))
        self.forward_duration_sec = float(rospy.get_param("~forward_duration_sec", 2.5))

        self.reverse_linear_speed_mm_s = int(rospy.get_param("~reverse_linear_speed_mm_s", 300))
        self.reverse_duration_sec = float(rospy.get_param("~reverse_duration_sec", 3.0))
        self.return_tolerance_mm = float(rospy.get_param("~return_tolerance_mm", 100.0))
        self.return_tolerance_hall = float(rospy.get_param("~return_tolerance_hall", 120.0))
        self.return_stop_buffer_mm = float(rospy.get_param("~return_stop_buffer_mm", 180.0))
        self.return_stop_buffer_hall = float(rospy.get_param("~return_stop_buffer_hall", 80.0))
        self.return_correction_speed_mm_s = int(rospy.get_param("~return_correction_speed_mm_s", 120))
        self.return_correction_timeout_sec = float(rospy.get_param("~return_correction_timeout_sec", 3.0))
        self.return_correction_max_attempts = int(rospy.get_param("~return_correction_max_attempts", 3))
        self.stop_speed_zero_count = int(rospy.get_param("~stop_speed_zero_count", 3))
        self.stop_query_timeout_sec = float(rospy.get_param("~stop_query_timeout_sec", 8.0))
        self.post_return_lateral_distance_mm = float(
            rospy.get_param(
                "~post_return_lateral_distance_mm",
                rospy.get_param(f"{self.workflow_param_ns}/lateral_distance_mm", -1750.0),
            )
        )
        self.post_return_lateral_speed_hall_s = int(rospy.get_param("~post_return_lateral_speed_hall_s", 32))
        self.post_return_lateral_timeout_sec = float(rospy.get_param("~post_return_lateral_timeout_sec", 12.0))

        self.angular_speed_cmd = int(rospy.get_param("~angular_speed_cmd", 0))
        self.stop_settle_sec = float(rospy.get_param("~stop_settle_sec", 0.4))
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", 20.0))

        # ========== 运行状态 ==========
        self.can_bus = None
        self.current_mode = None
        self.running = True
        self.sequence_busy = False
        self.trigger_requested = False
        self.trigger_lock = threading.Lock()
        self.ctrl_rx_lock = threading.Lock()
        self.wheel_odom_lock = threading.Lock()
        self.ctrl_rx_queue = deque(maxlen=128)
        self.can_rx_thread = None
        self.can_rx_running = False
        self.wheel_odom_frames = {wheel_id: None for wheel_id in self.wheel_odom_ids}
        self.logged_wheel_odom_ids = set()
        self.warned_missing_wheel_scale = False
        self.sequence_counter = 0

        self.start_odom_hall = None
        self.last_odom_hall = None
        self.start_odom_mm = None
        self.last_odom_mm = None
        self.forward_end_odom_hall = None
        self.forward_end_odom_mm = None

        self._init_can()

        self.flag_sub = rospy.Subscriber(
            self.flag_topic,
            Int32,
            self.flag_callback,
            queue_size=1,
        )
        self.done_pub = rospy.Publisher(
            self.done_topic,
            Int32,
            queue_size=4,
            latch=False,
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("=" * 70)
        rospy.loginfo("上轨道往返控制脚本已启动")
        rospy.loginfo(f"  标志位话题: {self.flag_topic}")
        rospy.loginfo(f"  触发值: {self.trigger_value}")
        rospy.loginfo(f"  完成信号话题: {self.done_topic}")
        rospy.loginfo(f"  流程配置命名空间: {self.workflow_param_ns}")
        rospy.loginfo(f"  计划循环次数: {self.planned_cycle_count}")
        rospy.loginfo(f"  行驶模式: 0x{self.MODE_DRIVE:02X}")
        rospy.loginfo(f"  等待模式: 0x{self.MODE_LATERAL:02X}")
        rospy.loginfo(f"  轮式里程计ID: {list(self.wheel_odom_ids)}")
        if self.wheel_hall_to_mm > 0:
            rospy.loginfo(f"  轮式里程计换算: 1 hall = {self.wheel_hall_to_mm:.6f} mm")
        else:
            rospy.logwarn("  未配置 ~wheel_hall_to_mm，当前仅缓存四轮霍尔值，距离判断仍会按时间兜底")
        rospy.loginfo(f"  前进速度: {self.forward_linear_speed_mm_s} mm/s")
        rospy.loginfo(f"  倒车固定速度: {self.reverse_linear_speed_mm_s} mm/s")
        rospy.loginfo(f"  回零停车缓冲: {self.return_stop_buffer_mm:.1f} mm")
        rospy.loginfo(f"  回零修正速度: {self.return_correction_speed_mm_s} mm/s")
        rospy.loginfo(
            f"  回零后横移: distance={self.post_return_lateral_distance_mm:.1f} mm, "
            f"speed={self.post_return_lateral_speed_hall_s} hall/s"
        )
        rospy.loginfo("=" * 70)

    # ============================================================
    # 基础 CAN
    # ============================================================

    def _init_can(self):
        if can is None:
            rospy.logwarn("python-can 不可用，进入模拟模式")
            self.can_bus = None
            return

        try:
            self.can_bus = can.Bus(
                channel=self.can_channel,
                interface="socketcan",
                bitrate=self.can_bitrate,
            )
            filters = [{"can_id": self.can_id, "can_mask": 0x7FF, "extended": False}]
            for wheel_id in self.wheel_odom_ids:
                filters.append({"can_id": wheel_id, "can_mask": 0x7FF, "extended": False})
            self.can_bus.set_filters(filters)
            self.can_rx_running = True
            self.can_rx_thread = threading.Thread(target=self._can_rx_loop, daemon=True)
            self.can_rx_thread.start()
            rospy.loginfo(f"CAN 总线初始化成功: {self.can_channel}")
        except Exception as e:
            rospy.logerr(f"CAN 总线初始化失败: {e}，进入模拟模式")
            self.can_bus = None

    def _append_ctrl_frame(self, msg):
        with self.ctrl_rx_lock:
            self.ctrl_rx_queue.append(msg)

    def _pop_ctrl_frame(self):
        with self.ctrl_rx_lock:
            if self.ctrl_rx_queue:
                return self.ctrl_rx_queue.popleft()
        return None

    def _clear_ctrl_rx_queue(self):
        with self.ctrl_rx_lock:
            self.ctrl_rx_queue.clear()

    def _handle_wheel_odom_frame(self, msg):
        if len(msg.data) < 8 or msg.data[0] != 0x25:
            return False

        wheel_id = int(msg.arbitration_id)
        total_hall = int.from_bytes(bytes(msg.data[1:5]), byteorder="big", signed=False)
        power_on_hall = int.from_bytes(bytes(msg.data[5:8]), byteorder="big", signed=False)
        snapshot = {
            "wheel_id": wheel_id,
            "total_hall": total_hall,
            "power_on_hall": power_on_hall,
            "timestamp": time.time(),
        }

        with self.wheel_odom_lock:
            self.wheel_odom_frames[wheel_id] = snapshot

        if wheel_id not in self.logged_wheel_odom_ids:
            self.logged_wheel_odom_ids.add(wheel_id)
            rospy.loginfo(
                f"收到轮{wheel_id}里程计上报: total_hall={total_hall}, power_on_hall={power_on_hall}"
            )

        return True

    def _can_rx_loop(self):
        while self.can_rx_running and self.can_bus is not None and not rospy.is_shutdown():
            try:
                msg = self.can_bus.recv(timeout=0.05)
            except Exception as e:
                if self.can_rx_running and not rospy.is_shutdown():
                    rospy.logwarn(f"CAN 接收异常: {e}")
                    time.sleep(0.1)
                continue

            if msg is None or len(msg.data) == 0:
                continue

            arbitration_id = int(msg.arbitration_id)

            if arbitration_id in self.wheel_odom_id_set and msg.data[0] == 0x25:
                self._handle_wheel_odom_frame(msg)
                continue

            if arbitration_id == self.can_id:
                self._append_ctrl_frame(msg)

    def send_can_message(self, data):
        if self.can_bus is None or can is None:
            rospy.loginfo(f"[模拟CAN] {' '.join(f'{int(b):02X}' for b in data)}")
            return True

        try:
            msg = can.Message(
                arbitration_id=self.can_id,
                data=data,
                is_extended_id=False,
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            rospy.logerr(f"CAN 发送失败: {e}")
            return False

    def recv_cmd(self, cmd, timeout=0.3):
        if self.can_bus is None:
            mock_data = [cmd, 0, 0, 0, 0, self.current_mode or self.MODE_LATERAL, 1, 0]
            return type("MockMsg", (), {"arbitration_id": self.can_id, "data": mock_data})()

        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self._pop_ctrl_frame()
            if msg is None:
                time.sleep(0.01)
                continue
            if msg.data[0] == cmd:
                return msg
        return None

    def query_system_status(self, retry_count=3):
        if self.can_bus is None:
            return {
                "angle": 0,
                "speed": 0,
                "mode": self.current_mode if self.current_mode is not None else self.MODE_LATERAL,
                "arrived": True,
                "motion_state": 0x00,
            }

        for attempt in range(retry_count):
            try:
                self._clear_ctrl_rx_queue()
                if not self.send_can_message([0x26, 0, 0, 0, 0, 0, 0, 0]):
                    return None

                msg = self.recv_cmd(0x26, timeout=0.3)
                if msg is None:
                    if attempt < retry_count - 1:
                        time.sleep(0.1)
                        continue
                    return None

                return {
                    "angle": (msg.data[1] << 8) | msg.data[2],
                    "speed": (msg.data[3] << 8) | msg.data[4],
                    "mode": msg.data[5],
                    "arrived": (msg.data[6] == 0x01),
                    "motion_state": msg.data[7],
                }
            except Exception as e:
                if attempt == retry_count - 1:
                    rospy.logwarn(f"状态查询异常: {e}")

        return None

    def switch_mode_if_needed(self, target_mode):
        if self.current_mode == target_mode:
            return True

        if self.can_bus is None:
            self.current_mode = target_mode
            rospy.loginfo(f"[模拟] 模式切换 -> 0x{target_mode:02X}")
            return True

        status = self.query_system_status()
        if status is not None and status["mode"] == target_mode:
            self.current_mode = target_mode
            return True

        if not self.send_can_message([0x05, 0, 0, target_mode, 0, 0, 0, 0]):
            return False

        rospy.loginfo(f"等待模式切换到 0x{target_mode:02X}")
        time.sleep(self.MODE_SWITCH_WAIT_TIME)

        start_time = time.time()
        arrived_count = 0

        while time.time() - start_time < self.QUERY_TIMEOUT and not rospy.is_shutdown():
            status = self.query_system_status()
            if status is None:
                time.sleep(self.MODE_SWITCH_QUERY_INTERVAL)
                continue

            if status["mode"] != target_mode:
                time.sleep(self.MODE_SWITCH_QUERY_INTERVAL)
                continue

            if status["arrived"]:
                arrived_count += 1
                if arrived_count >= 2:
                    self.current_mode = target_mode
                    rospy.loginfo(f"模式切换完成 -> 0x{target_mode:02X}")
                    return True
            else:
                arrived_count = 0

            time.sleep(self.MODE_SWITCH_QUERY_INTERVAL)

        rospy.logwarn(f"模式切换超时 -> 0x{target_mode:02X}")
        return False

    # ============================================================
    # 0x0C 速度模式
    # ============================================================

    @staticmethod
    def _to_int16_bytes(value):
        value = int(max(-32768, min(32767, int(round(value)))))
        if value < 0:
            value = (1 << 16) + value
        return (value >> 8) & 0xFF, value & 0xFF

    @staticmethod
    def _mm_to_hall(distance_mm):
        return int(round(abs(distance_mm) / DEFAULT_WHEEL_HALL_TO_MM))

    def send_speed_mode_command(self, linear_mm_s, angular_cmd=0):
        """
        用户指定格式:
        0x0C 00 00 [linear_hi] [linear_lo] [angular_hi] [angular_lo] 00
        """
        linear_hi, linear_lo = self._to_int16_bytes(linear_mm_s)
        angular_hi, angular_lo = self._to_int16_bytes(angular_cmd)

        data = bytearray(8)
        data[0] = 0x0C
        data[1] = 0x00
        data[2] = 0x00
        data[3] = linear_hi
        data[4] = linear_lo
        data[5] = angular_hi
        data[6] = angular_lo
        data[7] = 0x00

        rospy.loginfo(
            f"发送速度命令: linear={int(linear_mm_s)} mm/s, angular={int(angular_cmd)}"
        )
        return self.send_can_message(data)

    def stop_speed_motion(self):
        if self.current_mode == self.MODE_LATERAL:
            rospy.loginfo("当前为横移模式，跳过 0x0C 速度停止命令")
            return True
        return self.send_speed_mode_command(0, 0)

    def send_lateral_position_command(self, distance_mm, speed_hall):
        if self.can_bus is None or can is None:
            rospy.loginfo(f"[模拟CAN] 横移位置命令: distance={distance_mm:.1f} mm, speed={speed_hall} hall/s")
            return True

        distance_hall = max(0, min(0xFFFF, self._mm_to_hall(distance_mm)))
        speed_hall = max(1, min(0xFFFF, int(abs(speed_hall))))
        direction_byte = 0x01 if distance_mm >= 0 else 0x02

        data = bytearray(8)
        data[0] = 0x0E
        data[1] = (distance_hall >> 8) & 0xFF
        data[2] = distance_hall & 0xFF
        data[3] = (speed_hall >> 8) & 0xFF
        data[4] = speed_hall & 0xFF
        data[5] = direction_byte
        data[6] = 0x00
        data[7] = 0x00

        try:
            msg = can.Message(
                arbitration_id=self.can_id,
                data=data,
                is_extended_id=False,
            )
            self.can_bus.send(msg)
            rospy.loginfo(
                f"发送横移位置命令: distance={distance_mm:.1f} mm ({distance_hall} hall), "
                f"speed={speed_hall} hall/s, direction=0x{direction_byte:02X}"
            )
            return True
        except Exception as e:
            rospy.logerr(f"横移位置命令发送失败: {e}")
            return False

    # ============================================================
    # 轮式里程计
    # ============================================================

    def query_all_wheel_odometry(self, fresh_timeout=None):
        """
        返回四个轮子的最新里程计缓存。

        每个轮子的值形如:
        {
            "wheel_id": 1,
            "total_hall": ...,
            "power_on_hall": ...,
            "timestamp": ...
        }

        fresh_timeout:
        - None: 不做新鲜度过滤,直接返回最后一次缓存
        - >0: 超过该秒数未更新的轮子返回 None
        """
        with self.wheel_odom_lock:
            snapshots = {
                wheel_id: (None if item is None else dict(item))
                for wheel_id, item in self.wheel_odom_frames.items()
            }

        if fresh_timeout is None or fresh_timeout <= 0:
            return snapshots

        now = time.time()
        filtered = {}
        for wheel_id, item in snapshots.items():
            if item is None:
                filtered[wheel_id] = None
                continue

            if now - item["timestamp"] > fresh_timeout:
                filtered[wheel_id] = None
            else:
                filtered[wheel_id] = item

        return filtered

    def query_wheel_odometry_hall(self):
        """
        查询四个轮子的当前里程计,并返回整车平均霍尔值。

        默认使用 data[5:8] 的本次上电霍尔值;
        若 ~wheel_odom_use_total_hall=true, 则改用总霍尔值。
        """
        snapshots = self.query_all_wheel_odometry(fresh_timeout=self.wheel_odom_timeout_sec)
        hall_key = "total_hall" if self.wheel_odom_use_total_hall else "power_on_hall"
        hall_values = [float(item[hall_key]) for item in snapshots.values() if item is not None]

        if not hall_values:
            return None

        return sum(hall_values) / len(hall_values)

    def get_relative_distance_hall(self):
        if self.start_odom_hall is None:
            return None

        current_hall = self.query_wheel_odometry_hall()
        self.last_odom_hall = current_hall
        if current_hall is None:
            return None

        return float(current_hall - self.start_odom_hall)

    def _wheel_metric_name(self):
        return "total_hall" if self.wheel_odom_use_total_hall else "power_on_hall"

    def log_wheel_odometry_snapshot(self, prefix):
        snapshots = self.query_all_wheel_odometry(fresh_timeout=self.wheel_odom_timeout_sec)
        metric_name = self._wheel_metric_name()
        valid_items = [(wheel_id, item) for wheel_id, item in sorted(snapshots.items()) if item is not None]
        if not valid_items:
            rospy.logwarn(f"{prefix}: 当前没有有效四轮里程计缓存")
            return

        desc = ", ".join(f"轮{wheel_id}={int(item[metric_name])}" for wheel_id, item in valid_items)
        rospy.loginfo(f"{prefix}: {desc}")

    def wait_until_speed_zero(self, timeout=None, stable_count=None, phase_name="运动"):
        if self.can_bus is None:
            return True

        if timeout is None:
            timeout = self.stop_query_timeout_sec
        if stable_count is None:
            stable_count = self.stop_speed_zero_count

        start_time = time.time()
        zero_count = 0

        while time.time() - start_time < timeout and not rospy.is_shutdown():
            status = self.query_system_status()
            if status is None:
                time.sleep(self.MOTION_QUERY_INTERVAL)
                continue

            speed = status["speed"]
            rospy.loginfo_throttle(0.5, f"{phase_name}: 当前反馈速度={speed}")

            if speed == 0:
                zero_count += 1
                if zero_count >= stable_count:
                    rospy.loginfo(f"{phase_name}: 已稳定检测到速度为 0")
                    return True
            else:
                zero_count = 0

            time.sleep(self.MOTION_QUERY_INTERVAL)

        rospy.logwarn(f"{phase_name}: 等待速度归零超时")
        return False

    def wait_position_motion_complete(
        self,
        phase_name,
        timeout,
        expected_duration=None,
        allow_implicit_complete=False,
    ):
        if self.can_bus is None:
            if expected_duration is not None:
                time.sleep(min(max(expected_duration, 0.2), 1.0))
            return True

        start_time = time.time()
        motion_started = False
        zero_count = 0
        stable_needed = 3
        min_wait = 0.8
        initial_status = self.query_system_status()
        initial_motion_state = None if initial_status is None else initial_status.get("motion_state")
        if expected_duration is not None:
            min_wait = max(0.8, expected_duration * 0.5)

        motion_start_deadline = time.time() + 3.0
        while time.time() < motion_start_deadline and not rospy.is_shutdown():
            status = self.query_system_status()
            if status is None:
                time.sleep(0.05)
                continue

            speed = status["speed"]
            motion_state = status.get("motion_state")
            if (
                speed > 0 or
                not status["arrived"] or
                (initial_motion_state is not None and motion_state != initial_motion_state)
            ):
                motion_started = True
                rospy.loginfo(
                    f"{phase_name}: 检测到动作开始, speed={speed}, "
                    f"arrived={status['arrived']}, motion_state=0x{motion_state:02X}"
                )
                break
            time.sleep(0.05)

        if not motion_started:
            rospy.logwarn(
                f"{phase_name}: 未明确检测到动作开始，继续按完成态等待"
                + (" (允许隐式完成)" if allow_implicit_complete else "")
            )

        while time.time() - start_time < timeout and not rospy.is_shutdown():
            status = self.query_system_status()
            if status is None:
                time.sleep(self.MOTION_QUERY_INTERVAL)
                continue

            speed = status["speed"]
            arrived = status["arrived"]
            motion_state = status.get("motion_state")
            rospy.loginfo_throttle(
                0.5,
                f"{phase_name}: mode=0x{status['mode']:02X}, speed={speed}, "
                f"arrived={arrived}, motion_state=0x{motion_state:02X}, "
                f"started={motion_started}"
            )

            if (
                speed > 0 or
                not arrived or
                (initial_motion_state is not None and motion_state != initial_motion_state)
            ):
                motion_started = True
                zero_count = 0
            elif speed == 0 and arrived and time.time() - start_time >= min_wait:
                zero_count += 1
                if zero_count >= stable_needed:
                    if motion_started:
                        rospy.loginfo(f"{phase_name}: 动作完成")
                        return True
                    if allow_implicit_complete:
                        rospy.logwarn(
                            f"{phase_name}: 未捕获到明确启动信号，但已稳定处于完成态，按隐式完成处理"
                        )
                        return True
            else:
                zero_count = 0

            time.sleep(self.MOTION_QUERY_INTERVAL)

        rospy.logwarn(f"{phase_name}: 等待动作完成超时")
        return False

    def wait_for_wheel_odometry(self, timeout=None):
        """
        等待四轮里程计至少收到一轮有效上报。
        """
        if timeout is None:
            timeout = self.wheel_odom_timeout_sec

        deadline = time.time() + max(0.0, timeout)
        snapshots = self.query_all_wheel_odometry(fresh_timeout=timeout)
        while time.time() < deadline and not rospy.is_shutdown():
            if all(item is not None for item in snapshots.values()):
                break
            time.sleep(0.02)
            snapshots = self.query_all_wheel_odometry(fresh_timeout=timeout)

        return snapshots

    def query_wheel_odometry_mm(self):
        """
        基于四轮 0x25 里程计上报,返回整车平均里程(mm)。

        需要额外配置:
        - ~wheel_hall_to_mm: 单个霍尔对应多少 mm
        """
        avg_hall = self.query_wheel_odometry_hall()
        self.last_odom_hall = avg_hall
        if avg_hall is None:
            return None

        if self.wheel_hall_to_mm <= 0:
            if not self.warned_missing_wheel_scale:
                rospy.logwarn("已收到四轮霍尔值,但未配置 ~wheel_hall_to_mm，距离判断继续按时间兜底")
                self.warned_missing_wheel_scale = True
            return None

        return avg_hall * self.wheel_hall_to_mm

    def get_cumulative_odometry_state(self, preferred_unit=None):
        if preferred_unit == "hall":
            current_hall = self.query_wheel_odometry_hall()
            if current_hall is not None:
                return {"value": float(current_hall), "unit": "hall"}
        elif preferred_unit == "mm":
            current_mm = self.query_wheel_odometry_mm()
            if current_mm is not None:
                return {"value": float(current_mm), "unit": "mm"}

        current_mm = self.query_wheel_odometry_mm()
        if current_mm is not None:
            return {"value": float(current_mm), "unit": "mm"}

        current_hall = self.query_wheel_odometry_hall()
        if current_hall is not None:
            return {"value": float(current_hall), "unit": "hall"}

        return None

    def record_start_odometry(self):
        self.forward_end_odom_hall = None
        self.forward_end_odom_mm = None
        snapshots = self.wait_for_wheel_odometry(timeout=self.wheel_odom_timeout_sec)
        valid_snapshots = {wheel_id: item for wheel_id, item in snapshots.items() if item is not None}
        if valid_snapshots:
            hall_key = self._wheel_metric_name()
            hall_desc = ", ".join(
                f"轮{wheel_id}={int(item[hall_key])}" for wheel_id, item in sorted(valid_snapshots.items())
            )
            rospy.loginfo(f"记录起点时四轮霍尔值: {hall_desc}")

        self.start_odom_hall = self.query_wheel_odometry_hall()
        self.last_odom_hall = self.start_odom_hall
        self.start_odom_mm = self.query_wheel_odometry_mm()
        self.last_odom_mm = self.start_odom_mm
        if self.start_odom_mm is None:
            rospy.logwarn("轮式里程计未接入，前进/回零将先按时间兜底")
        else:
            rospy.loginfo(f"记录起点里程计: {self.start_odom_mm:.1f} mm")

    def get_relative_distance_mm(self):
        if self.start_odom_mm is None:
            return None

        current_odom = self.query_wheel_odometry_mm()
        self.last_odom_mm = current_odom
        if current_odom is None:
            return None

        return float(current_odom - self.start_odom_mm)

    def record_forward_end_odometry(self):
        self.forward_end_odom_hall = self.query_wheel_odometry_hall()
        self.forward_end_odom_mm = self.query_wheel_odometry_mm()
        if self.forward_end_odom_mm is not None:
            rospy.loginfo(f"记录前进终点里程计: {self.forward_end_odom_mm:.1f} mm")
        elif self.forward_end_odom_hall is not None:
            rospy.loginfo(f"记录前进终点里程计: {self.forward_end_odom_hall:.1f} hall")
        else:
            rospy.logwarn("记录前进终点里程计失败: 当前无有效轮式里程计")

    def get_return_plan(self):
        if self.start_odom_mm is not None and self.forward_end_odom_mm is not None:
            return {
                "target_distance": max(0.0, float(self.forward_end_odom_mm - self.start_odom_mm)),
                "unit": "mm",
                "tolerance": float(self.return_tolerance_mm),
                "stop_buffer": max(float(self.return_tolerance_mm), float(self.return_stop_buffer_mm)),
            }

        if self.start_odom_hall is not None and self.forward_end_odom_hall is not None:
            return {
                "target_distance": max(0.0, float(self.forward_end_odom_hall - self.start_odom_hall)),
                "unit": "hall",
                "tolerance": float(self.return_tolerance_hall),
                "stop_buffer": max(float(self.return_tolerance_hall), float(self.return_stop_buffer_hall)),
            }

        return None

    def run_distance_segment(self, cmd_speed_mm_s, target_distance, unit, phase_name, stop_buffer=0.0, timeout=None):
        if target_distance <= 0:
            rospy.loginfo(f"{phase_name}: 目标距离为 0，跳过段运动")
            return 0.0

        start_state = self.get_cumulative_odometry_state(preferred_unit=unit)
        if start_state is None:
            rospy.logwarn(f"{phase_name}: 无法读取累计轮式里程计")
            return None

        unit = start_state["unit"]
        segment_start = start_state["value"]
        stop_threshold = max(0.0, float(target_distance) - max(0.0, float(stop_buffer)))

        if not self.send_speed_mode_command(cmd_speed_mm_s, self.angular_speed_cmd):
            return None

        if timeout is None:
            timeout = self.reverse_duration_sec

        start_time = time.time()
        while self.running and not rospy.is_shutdown():
            current_state = self.get_cumulative_odometry_state(preferred_unit=unit)
            if current_state is None:
                if time.time() - start_time >= timeout:
                    rospy.logwarn(f"{phase_name}: 读取累计轮式里程计超时")
                    break
                time.sleep(0.02)
                continue

            moved_distance = max(0.0, current_state["value"] - segment_start)
            rospy.loginfo_throttle(
                0.5,
                f"{phase_name}: moved_{unit}={moved_distance:.1f}, target_{unit}={target_distance:.1f}, "
                f"stop_threshold_{unit}={stop_threshold:.1f}"
            )

            if moved_distance >= stop_threshold:
                rospy.loginfo(
                    f"{phase_name}: 达到停车阈值, moved_{unit}={moved_distance:.1f}, "
                    f"target_{unit}={target_distance:.1f}"
                )
                break

            if time.time() - start_time >= timeout:
                rospy.logwarn(f"{phase_name}: 段运动超时")
                break

            time.sleep(0.02)

        self.stop_speed_motion()
        self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
        time.sleep(self.stop_settle_sec)

        end_state = self.get_cumulative_odometry_state(preferred_unit=unit)
        if end_state is None:
            rospy.logwarn(f"{phase_name}: 停车后无法读取累计轮式里程计")
            return None

        actual_distance = max(0.0, end_state["value"] - segment_start)
        rospy.loginfo(
            f"{phase_name}: 实际段距离={actual_distance:.1f} {unit}, "
            f"目标={target_distance:.1f} {unit}"
        )
        return actual_distance

    def estimate_segment_timeout(self, distance, unit, speed_mm_s, default_timeout):
        speed_mm_s = max(1.0, abs(float(speed_mm_s)))
        estimated_distance_mm = None

        if unit == "mm":
            estimated_distance_mm = float(distance)
        elif unit == "hall" and self.wheel_hall_to_mm > 0:
            estimated_distance_mm = float(distance) * self.wheel_hall_to_mm

        if estimated_distance_mm is None:
            return max(float(default_timeout), 1.0)

        estimated_time = estimated_distance_mm / speed_mm_s + 2.0
        return max(float(default_timeout), estimated_time)

    def execute_post_return_lateral_move(self):
        distance_mm = float(self.post_return_lateral_distance_mm)
        if abs(distance_mm) < 1e-6:
            rospy.loginfo("回零后横移距离为 0，跳过横移位置控制")
            return True

        status = self.query_system_status()
        if status is None:
            rospy.logwarn("横移前无法确认当前模式")
            return False

        if status["mode"] != self.MODE_LATERAL:
            rospy.logwarn(f"横移前模式不正确: 当前=0x{status['mode']:02X}, 目标=0x{self.MODE_LATERAL:02X}")
            return False

        speed_hall = max(1, abs(self.post_return_lateral_speed_hall_s))
        expected_duration = self._mm_to_hall(distance_mm) / float(speed_hall) + 1.0

        rospy.loginfo(
            f"开始执行回零后横移位置控制: distance={distance_mm:.1f} mm, "
            f"speed={speed_hall} hall/s, expected_duration={expected_duration:.1f} s"
        )

        if not self.send_lateral_position_command(distance_mm, speed_hall):
            return False

        return self.wait_position_motion_complete(
            phase_name="回零后横移",
            timeout=max(self.post_return_lateral_timeout_sec, expected_duration + 2.0),
            expected_duration=expected_duration,
            allow_implicit_complete=True,
        )

    def should_run_post_return_lateral_move(self, sequence_id):
        planned = int(self.planned_cycle_count)
        if planned == 0:
            return True
        if planned < 0:
            return False
        return sequence_id < planned

    def publish_cycle_done(self, sequence_id, success):
        if not hasattr(self, "done_pub") or self.done_pub is None:
            return

        value = int(sequence_id if success else -sequence_id)
        try:
            self.done_pub.publish(Int32(data=value))
            result = "成功" if success else "失败"
            rospy.loginfo(f"发布流程完成信号: value={value} ({result})")
        except Exception as e:
            rospy.logwarn(f"发布流程完成信号失败: {e}")

    # ============================================================
    # 触发与流程
    # ============================================================

    def flag_callback(self, msg):
        if int(msg.data) != self.trigger_value:
            return

        with self.trigger_lock:
            if self.sequence_busy or self.trigger_requested:
                return
            self.trigger_requested = True

        rospy.loginfo(f"收到上轨触发标志: {msg.data}")

    def run_forward_phase(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("阶段1: 切四轮转向模式后大速度上轨")
        rospy.loginfo("=" * 60)

        self.record_start_odometry()
        self.log_wheel_odometry_snapshot("前进前四轮里程计")

        if not self.send_speed_mode_command(self.forward_linear_speed_mm_s, self.angular_speed_cmd):
            return False

        start_time = time.time()
        while self.running and not rospy.is_shutdown():
            relative_hall = self.get_relative_distance_hall()
            relative_mm = self.get_relative_distance_mm()

            if relative_mm is not None:
                rospy.loginfo_throttle(0.5, f"前进中: relative_mm={relative_mm:.1f}, relative_hall={relative_hall}")
                if abs(relative_mm) >= self.forward_distance_mm:
                    rospy.loginfo(f"前进距离达到目标: {relative_mm:.1f} mm")
                    break
            elif relative_hall is not None and self.forward_distance_hall > 0:
                rospy.loginfo_throttle(0.5, f"前进中: relative_hall={relative_hall:.1f}")
                if abs(relative_hall) >= self.forward_distance_hall:
                    rospy.loginfo(f"前进霍尔值达到目标: {relative_hall:.1f} hall")
                    break
            else:
                if time.time() - start_time >= self.forward_duration_sec:
                    rospy.loginfo(f"前进时间达到兜底值: {self.forward_duration_sec:.2f}s")
                    break

            time.sleep(0.02)

        self.stop_speed_motion()
        self.wait_until_speed_zero(phase_name="前进停车")
        time.sleep(self.stop_settle_sec)
        self.record_forward_end_odometry()
        self.log_wheel_odometry_snapshot("前进停稳后四轮里程计")
        return True

    def run_reverse_phase(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("阶段2: 按累计轮式里程计倒车回到前进前位置")
        rospy.loginfo("=" * 60)

        plan = self.get_return_plan()
        if plan is None:
            rospy.logwarn("回程前无法构建累计里程计划，退回时间兜底")
            reverse_speed_cmd = -abs(self.reverse_linear_speed_mm_s)
            if not self.send_speed_mode_command(reverse_speed_cmd, self.angular_speed_cmd):
                return False

            start_time = time.time()
            while self.running and not rospy.is_shutdown():
                if time.time() - start_time >= self.reverse_duration_sec:
                    rospy.loginfo(f"倒车时间达到兜底值: {self.reverse_duration_sec:.2f}s")
                    break
                time.sleep(0.02)

            self.stop_speed_motion()
            self.wait_until_speed_zero(phase_name="倒车停车")
            time.sleep(self.stop_settle_sec)
            self.log_wheel_odometry_snapshot("回零停稳后四轮里程计")
            return True

        self.log_wheel_odometry_snapshot("回程前四轮里程计")
        rospy.loginfo(
            f"回程累计计划: target_{plan['unit']}={plan['target_distance']:.1f}, "
            f"stop_buffer_{plan['unit']}={plan['stop_buffer']:.1f}, tolerance_{plan['unit']}={plan['tolerance']:.1f}"
        )

        remaining_distance = float(plan["target_distance"])
        main_timeout = self.estimate_segment_timeout(
            distance=remaining_distance,
            unit=plan["unit"],
            speed_mm_s=self.reverse_linear_speed_mm_s,
            default_timeout=self.reverse_duration_sec,
        )
        main_distance = self.run_distance_segment(
            cmd_speed_mm_s=-abs(self.reverse_linear_speed_mm_s),
            target_distance=remaining_distance,
            unit=plan["unit"],
            phase_name="倒车主段",
            stop_buffer=plan["stop_buffer"],
            timeout=main_timeout,
        )
        if main_distance is None:
            return False

        remaining_distance -= main_distance
        rospy.loginfo(
            f"倒车主段结束: moved_{plan['unit']}={main_distance:.1f}, remaining_{plan['unit']}={remaining_distance:.1f}"
        )

        attempt = 0
        while abs(remaining_distance) > plan["tolerance"] and attempt < self.return_correction_max_attempts:
            attempt += 1
            correction_speed = max(1, abs(self.return_correction_speed_mm_s))
            cmd_speed = -correction_speed if remaining_distance > 0 else correction_speed
            phase_name = f"回零修正{attempt}"
            rospy.logwarn(
                f"{phase_name}: remaining_{plan['unit']}={remaining_distance:.1f}, cmd_speed={cmd_speed} mm/s"
            )

            correction_timeout = self.estimate_segment_timeout(
                distance=abs(remaining_distance),
                unit=plan["unit"],
                speed_mm_s=correction_speed,
                default_timeout=self.return_correction_timeout_sec,
            )
            correction_distance = self.run_distance_segment(
                cmd_speed_mm_s=cmd_speed,
                target_distance=abs(remaining_distance),
                unit=plan["unit"],
                phase_name=phase_name,
                stop_buffer=plan["tolerance"],
                timeout=correction_timeout,
            )
            if correction_distance is None:
                return False

            if remaining_distance > 0:
                remaining_distance -= correction_distance
            else:
                remaining_distance += correction_distance

            rospy.loginfo(
                f"{phase_name}: moved_{plan['unit']}={correction_distance:.1f}, "
                f"remaining_{plan['unit']}={remaining_distance:.1f}"
            )

        self.log_wheel_odometry_snapshot("回零停稳后四轮里程计")
        if abs(remaining_distance) > plan["tolerance"]:
            rospy.logwarn(
                f"倒车回零后仍未到容差内: remaining_{plan['unit']}={remaining_distance:.1f}, "
                f"tolerance_{plan['unit']}={plan['tolerance']:.1f}"
            )
            return False

        rospy.loginfo(
            f"倒车回零完成: remaining_{plan['unit']}={remaining_distance:.1f}, "
            f"tolerance_{plan['unit']}={plan['tolerance']:.1f}"
        )
        return True

    def execute_sequence(self):
        success = False
        with self.trigger_lock:
            self.sequence_busy = True
            self.sequence_counter += 1
            sequence_id = self.sequence_counter

        try:
            rospy.loginfo("\n" + "=" * 70)
            rospy.loginfo(f"开始执行上轨往返流程，第 {sequence_id} 轮")
            rospy.loginfo("=" * 70)

            if not self.switch_mode_if_needed(self.MODE_DRIVE):
                rospy.logerr("切换到行驶模式失败")
                return False

            if not self.run_forward_phase():
                rospy.logerr("前进阶段失败")
                return False

            if not self.run_reverse_phase():
                rospy.logerr("倒车阶段失败")
                return False

            rospy.loginfo("阶段3: 切回等待模式")
            if not self.switch_mode_if_needed(self.MODE_LATERAL):
                rospy.logwarn("切回等待模式失败")
                return False

            if self.should_run_post_return_lateral_move(sequence_id):
                rospy.loginfo("阶段4: 确认横移模式并执行横移位置控制")
                if not self.execute_post_return_lateral_move():
                    rospy.logwarn("回零后横移位置控制失败")
                    return False
            else:
                rospy.loginfo("阶段4: 当前已是最后一轮，回零后不再执行横移")

            rospy.loginfo("上轨往返+横移流程完成，继续等待下一次标志位")
            success = True
            return True

        finally:
            self.stop_speed_motion()
            with self.trigger_lock:
                self.sequence_busy = False
                self.trigger_requested = False
            self.publish_cycle_done(sequence_id, success)

    def run(self):
        rate = rospy.Rate(max(1.0, self.loop_rate_hz))

        rospy.loginfo("启动后先切到等待模式")
        self.switch_mode_if_needed(self.MODE_LATERAL)

        while not rospy.is_shutdown() and self.running:
            should_run = False
            with self.trigger_lock:
                if self.trigger_requested and not self.sequence_busy:
                    should_run = True

            if should_run:
                self.execute_sequence()

            rate.sleep()

    def shutdown(self):
        if not self.running:
            return

        self.running = False
        self.can_rx_running = False
        self.stop_speed_motion()

        if self.can_rx_thread is not None and self.can_rx_thread.is_alive():
            self.can_rx_thread.join(timeout=0.2)

        if self.can_bus is not None:
            try:
                self.can_bus.shutdown()
            except Exception:
                pass


def signal_handler(sig, frame):
    rospy.signal_shutdown("用户终止")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    controller = UpRailFlagController()
    controller.run()


if __name__ == "__main__":
    main()
