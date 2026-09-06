#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上轨道往返状态机

流程:
1. 启动后默认切到等待模式(默认横移模式 0x04)
2. 等待 ROS 标志位触发
3. 启动后收到首个有效 FAST-LIVO yaw 时立即记录为首次姿态基准
4. 切到行驶模式(默认 0x02)
5. 记录起点 FAST 位姿
6. 大速度直行上轨(FAST 距离闭环判停)
7. 停车后按 FAST 位姿主闭环倒车回零
8. 若后面还有下一轮,切到横移模式并执行横移位置控制
9. 回零/横移结束后,用 SLAM 位姿角把车头恢复到第一次记录的基准角
10. 切回等待模式,通过完成信号话题上报本轮结果

说明:
- 速度命令按用户要求使用 0x0C 00 00 [线速度hi lo] [角速度hi lo] 00
- 线速度单位先按 mm/s
- 角速度当前默认给 0
- 当前默认使用 /aft_mapped_to_init 的 pose.pose.orientation 提取 yaw, 作为 FAST-LIVO 姿态角
- 当前版本已去除轮式里程计运行依赖，前进/回零均以 FAST 位姿为主
"""

import sys
import os
import time
import signal
import math
import threading
from collections import deque

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

try:
    import can
except ImportError:
    can = None


# 与 demo/yes_3_fix_3.py 中的 UnitConverter.HALL_TO_MM 保持一致
DEFAULT_WHEEL_HALL_TO_MM = 2.38
DEFAULT_ANGLE_DEG_TO_CNT = 4.18


class UpRailFlagController:
    def __init__(self):
        rospy.init_node(os.environ.get("RAIL_CONTROLLER_NODE_NAME", "yes_3_fix_3_uprail_flag"), anonymous=True)

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
        self.MODE_ANGLE = int(rospy.get_param("~angle_mode_code", 0x03))
        self.MODE_LATERAL = int(rospy.get_param("~wait_mode_code", 0x04))
        self.MODE_SWITCH_QUERY_INTERVAL = float(rospy.get_param("~mode_switch_query_interval", 0.15))
        self.MODE_SWITCH_WAIT_TIME = float(rospy.get_param("~mode_switch_wait_time", 3.0))
        self.MOTION_QUERY_INTERVAL = float(rospy.get_param("~motion_query_interval", 0.25))
        self.QUERY_TIMEOUT = float(rospy.get_param("~query_timeout", 10.0))
        self.angle_deg_to_cnt = float(rospy.get_param("~angle_deg_to_cnt", DEFAULT_ANGLE_DEG_TO_CNT))

        # ========== 触发标志位 ==========
        self.flag_topic = rospy.get_param("~flag_topic", "/flag1")
        self.trigger_value = int(rospy.get_param("~trigger_value", 1))
        self.done_topic = rospy.get_param("~done_topic", "/rail_cycle_done")
        self.workflow_param_ns = rospy.get_param("~workflow_param_ns", "/rail_workflow")
        # 固定使用 heading_topic 作为姿态源，避免旧参数 ~imu_topic 误覆盖
        self.heading_topic = rospy.get_param("~heading_topic", "/aft_mapped_to_init")
        self.heading_timeout_sec = float(
            rospy.get_param("~heading_timeout_sec", rospy.get_param("~imu_timeout_sec", 1.0))
        )
        self.use_external_heading_reference = bool(
            rospy.get_param(
                "~use_external_heading_reference",
                rospy.get_param(f"{self.workflow_param_ns}/use_external_heading_reference", True),
            )
        )
        self.capture_initial_heading_before_first_cycle = bool(
            rospy.get_param(
                "~capture_initial_heading_before_first_cycle",
                rospy.get_param(
                    f"{self.workflow_param_ns}/capture_initial_heading_before_first_cycle",
                    True,
                ),
            )
        )
        self.external_heading_reference_param = rospy.get_param(
            "~external_heading_reference_param",
            rospy.get_param(
                f"{self.workflow_param_ns}/external_heading_reference_param",
                f"{self.workflow_param_ns}/heading_reference",
            ),
        )
        self.external_heading_reference_max_age_sec = float(
            rospy.get_param(
                "~external_heading_reference_max_age_sec",
                rospy.get_param(
                    f"{self.workflow_param_ns}/external_heading_reference_max_age_sec",
                    600.0,
                ),
            )
        )
        self.planned_cycle_count = int(
            rospy.get_param(
                "~planned_cycle_count",
                rospy.get_param(f"{self.workflow_param_ns}/cycle_count", 2),
            )
        )

        # ========== 运动参数 ==========
        self.forward_linear_speed_mm_s = int(
            rospy.get_param(
                "~forward_linear_speed_mm_s",
                rospy.get_param(f"{self.workflow_param_ns}/forward_linear_speed_mm_s", 800),
            )
        )
        self.forward_distance_mm = float(rospy.get_param("~forward_distance_mm", 5000.0))
        self.forward_pick_step_mm = max(1.0, float(rospy.get_param("~forward_pick_step_mm", 500.0)))
        self.forward_pick_direct_finish_threshold_mm = max(
            0.0,
            float(rospy.get_param("~forward_pick_direct_finish_threshold_mm", 800.0)),
        )
        self.harvest_status_can_id = int(rospy.get_param("~harvest_status_can_id", 0x20))
        self.harvest_done_can_id = int(rospy.get_param("~harvest_done_can_id", 0x21))
        self.harvest_wait_timeout_sec = float(rospy.get_param("~harvest_wait_timeout_sec", 0.0))
        self.harvest_arm_delay_sec = max(0.0, float(rospy.get_param("~harvest_arm_delay_sec", 0.10)))
        self.harvest_rearm_quiet_sec = max(0.0, float(rospy.get_param("~harvest_rearm_quiet_sec", 0.30)))
        self.harvest_duplicate_suppress_sec = max(
            0.0,
            float(rospy.get_param("~harvest_duplicate_suppress_sec", 1.0)),
        )
        self.harvest_ready_payload = bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.harvest_busy_payload = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.harvest_done_payload = bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.forward_distance_hall = float(rospy.get_param("~forward_distance_hall", 0.0))
        self.forward_duration_sec = float(rospy.get_param("~forward_duration_sec", 2.5))

        self.reverse_linear_speed_mm_s = int(rospy.get_param("~reverse_linear_speed_mm_s", 300))
        self.reverse_duration_sec = float(rospy.get_param("~reverse_duration_sec", 3.0))
        self.return_tolerance_mm = float(rospy.get_param("~return_tolerance_mm", 100.0))
        self.return_tolerance_hall = float(rospy.get_param("~return_tolerance_hall", 120.0))
        self.return_stop_buffer_mm = float(rospy.get_param("~return_stop_buffer_mm", 180.0))
        self.return_stop_buffer_hall = float(rospy.get_param("~return_stop_buffer_hall", 80.0))
        self.return_correction_speed_mm_s = int(rospy.get_param("~return_correction_speed_mm_s", 60))
        self.return_correction_timeout_sec = float(rospy.get_param("~return_correction_timeout_sec", 3.0))
        self.return_correction_max_attempts = int(rospy.get_param("~return_correction_max_attempts", 3))
        self.stop_speed_zero_count = int(rospy.get_param("~stop_speed_zero_count", 3))
        self.stop_query_timeout_sec = float(rospy.get_param("~stop_query_timeout_sec", 8.0))
        self.stop_speed_zero_deadband = max(
            0,
            int(rospy.get_param("~stop_speed_zero_deadband", 35)),
        )
        self.return_use_fast_position = bool(rospy.get_param("~return_use_fast_position", True))
        self.return_allow_wheel_fallback = bool(rospy.get_param("~return_allow_wheel_fallback", True))
        self.return_fast_tolerance_mm = float(
            rospy.get_param("~return_fast_tolerance_mm", self.return_tolerance_mm)
        )
        self.return_fast_stop_lead_mm = max(
            0.0,
            float(rospy.get_param("~return_fast_stop_lead_mm", 50.0)),
        )
        self.return_fast_slowdown_distance_mm = float(
            rospy.get_param(
                "~return_fast_slowdown_distance_mm",
                max(600.0, self.return_fast_tolerance_mm * 6.0),
            )
        )
        self.return_fast_timeout_sec = float(rospy.get_param("~return_fast_timeout_sec", 0.0))
        self.return_fast_progress_window_sec = float(
            rospy.get_param("~return_fast_progress_window_sec", 2.0)
        )
        self.return_fast_min_progress_mm = float(rospy.get_param("~return_fast_min_progress_mm", 80.0))
        self.return_fast_no_progress_limit = int(rospy.get_param("~return_fast_no_progress_limit", 3))
        self.return_fast_move_away_abort_mm = float(
            rospy.get_param("~return_fast_move_away_abort_mm", 1200.0)
        )
        self.return_wheel_assist_enabled = bool(rospy.get_param("~return_wheel_assist_enabled", True))
        self.return_wheel_min_progress_mm = float(rospy.get_param("~return_wheel_min_progress_mm", 60.0))
        self.return_wheel_no_progress_limit = int(rospy.get_param("~return_wheel_no_progress_limit", 3))
        self.return_fast_wheel_gap_warn_mm = float(rospy.get_param("~return_fast_wheel_gap_warn_mm", 1500.0))
        self.return_fast_wheel_gap_abort_mm = float(rospy.get_param("~return_fast_wheel_gap_abort_mm", 3000.0))
        self.post_return_lateral_distance_mm = float(
            rospy.get_param(
                "~post_return_lateral_distance_mm",
                rospy.get_param(f"{self.workflow_param_ns}/lateral_distance_mm", -1750.0),
            )
        )
        self.post_return_lateral_move_count = self._load_optional_int_param(
            "~post_return_lateral_move_count",
            f"{self.workflow_param_ns}/lateral_move_count",
        )
        self.post_return_lateral_distances_mm = self._load_lateral_distances_param(
            "~post_return_lateral_distances_mm",
            f"{self.workflow_param_ns}/lateral_distances_mm",
        )
        if self.post_return_lateral_move_count is not None and self.post_return_lateral_move_count < 0:
            rospy.logwarn(
                f"横移次数配置为负数({self.post_return_lateral_move_count})，已按 0 处理"
            )
            self.post_return_lateral_move_count = 0
        if (
            self.post_return_lateral_move_count is None
            and len(self.post_return_lateral_distances_mm) > 0
        ):
            self.post_return_lateral_move_count = len(self.post_return_lateral_distances_mm)
            rospy.loginfo(
                f"检测到横移距离列表，将横移次数自动设置为 {self.post_return_lateral_move_count}"
            )
        self.post_return_lateral_speed_hall_s = int(rospy.get_param("~post_return_lateral_speed_hall_s", 32))
        self.post_return_lateral_timeout_sec = float(rospy.get_param("~post_return_lateral_timeout_sec", 12.0))
        self.angle_restore_speed_degps = float(rospy.get_param("~angle_restore_speed_degps", 2.0))
        self.angle_restore_tolerance_deg = float(rospy.get_param("~angle_restore_tolerance_deg", 1.0))
        self.angle_restore_timeout_sec = float(rospy.get_param("~angle_restore_timeout_sec", 15.0))
        self.angle_restore_max_attempts = int(rospy.get_param("~angle_restore_max_attempts", 2))
        self.angle_restore_heading_kp = float(rospy.get_param("~angle_restore_heading_kp", 1.0))
        self.angle_restore_min_speed_degps = float(rospy.get_param("~angle_restore_min_speed_degps", 0.6))
        self.angle_restore_max_speed_degps = float(
            rospy.get_param("~angle_restore_max_speed_degps", max(2.0, self.angle_restore_speed_degps))
        )
        self.angle_restore_command_step_deg = float(
            rospy.get_param("~angle_restore_direct_command_limit_deg", 45.0)
        )
        self.angle_restore_command_interval_sec = float(
            rospy.get_param("~angle_restore_command_interval_sec", 0.35)
        )
        self.angle_restore_hold_interval_sec = float(
            rospy.get_param("~angle_restore_hold_interval_sec", 0.25)
        )
        self.angle_restore_stabilize_count = int(rospy.get_param("~angle_restore_stabilize_count", 6))
        self.angle_restore_poll_hz = float(rospy.get_param("~angle_restore_poll_hz", 10.0))

        # 保留参数兼容，但运行时彻底停用轮式里程计路径，统一走 FAST 主线。
        self.use_wheel_odometry = False
        self.return_allow_wheel_fallback = False
        self.return_wheel_assist_enabled = False

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
        self.harvest_rx_lock = threading.Lock()
        self.wheel_odom_lock = threading.Lock()
        self.heading_lock = threading.Lock()
        self.ctrl_rx_queue = deque(maxlen=128)
        self.harvest_rx_queue = deque(maxlen=64)
        self.harvest_ready_sent_wall_time = 0.0
        self.harvest_done_ignore_until = 0.0
        self.harvest_last_rx_wall_time = 0.0
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
        self.start_fast_position = None
        self.start_fast_yaw_deg = None
        self.forward_end_fast_position = None
        self.forward_end_fast_yaw_deg = None
        self.heading_reference_deg = None
        self.heading_reference_source = None
        self.heading_reference_capture_wall_time = 0.0
        self.current_cycle_pre_uprail_angle_deg = None
        self.heading_last_yaw_deg = None
        self.heading_last_ros_stamp = None
        self.heading_last_recv_wall_time = 0.0
        self.heading_last_position = None
        self.logged_lateral_distance_fallback_indices = set()

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
        self.heading_sub = rospy.Subscriber(
            self.heading_topic,
            Odometry,
            self.heading_callback,
            queue_size=20,
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("=" * 70)
        rospy.loginfo(os.environ.get("RAIL_CONTROLLER_STARTUP_TITLE", "上轨道往返控制脚本已启动"))
        rospy.loginfo(f"  标志位话题: {self.flag_topic}")
        rospy.loginfo(f"  触发值: {self.trigger_value}")
        rospy.loginfo(f"  完成信号话题: {self.done_topic}")
        rospy.loginfo(f"  流程配置命名空间: {self.workflow_param_ns}")
        rospy.loginfo(f"  计划循环次数: {self.planned_cycle_count}")
        rospy.loginfo(f"  行驶模式: 0x{self.MODE_DRIVE:02X}")
        rospy.loginfo(f"  角度模式: 0x{self.MODE_ANGLE:02X}")
        rospy.loginfo(f"  等待模式: 0x{self.MODE_LATERAL:02X}")
        rospy.loginfo("  轮式里程计: 已停用，本版仅保留 FAST 位姿主线")
        rospy.loginfo(f"  前进速度: {self.forward_linear_speed_mm_s} mm/s")
        rospy.loginfo(f"  倒车固定速度: {self.reverse_linear_speed_mm_s} mm/s")
        rospy.loginfo(f"  回零补偿速度: {self.return_correction_speed_mm_s} mm/s")
        rospy.loginfo(f"  前进判停: FAST 距离闭环 target={self.forward_distance_mm:.1f} mm")
        rospy.loginfo(
            f"  前进采摘分段: step={self.forward_pick_step_mm:.1f} mm, "
            f"direct_finish_threshold={self.forward_pick_direct_finish_threshold_mm:.1f} mm, "
            f"harvest_status_can=0x{self.harvest_status_can_id:03X}, "
            f"harvest_done_can=0x{self.harvest_done_can_id:03X}, "
            f"arm_delay={self.harvest_arm_delay_sec:.2f}s, "
            f"rearm_quiet={self.harvest_rearm_quiet_sec:.2f}s, "
            f"duplicate_suppress={self.harvest_duplicate_suppress_sec:.2f}s"
        )
        rospy.loginfo(
            f"  回零主策略: FAST位置主闭环={'启用' if self.return_use_fast_position else '关闭'}, "
            f"tolerance={self.return_fast_tolerance_mm:.1f} mm"
        )
        rospy.loginfo(
            f"  回零减速切换距离: {self.return_fast_slowdown_distance_mm:.1f} mm "
            f"(进入该范围后改用补偿速度)"
        )
        rospy.loginfo(
            f"  回零细调提前停车余量: {self.return_fast_stop_lead_mm:.1f} mm"
        )
        rospy.loginfo(
            f"  停车速度死区: abs(speed)<={self.stop_speed_zero_deadband} 按速度 0 处理"
        )
        rospy.loginfo("  FAST不可用时轮里程回退: 已禁用")
        rospy.loginfo(
            f"  外部基准角读取: {'启用' if self.use_external_heading_reference else '禁用'}, "
            f"param={self.external_heading_reference_param}, "
            f"max_age={self.external_heading_reference_max_age_sec:.1f}s"
        )
        rospy.loginfo(
            f"  启动前置基准角锁存: "
            f"{'启用' if self.capture_initial_heading_before_first_cycle else '禁用'}"
        )
        rospy.loginfo("  回零超时/时间进展保护: 已禁用，持续按 FAST 距离闭环")
        rospy.loginfo("  轮里程计辅助保护: 已禁用")
        rospy.loginfo(
            f"  回零后横移默认值: distance={self.post_return_lateral_distance_mm:.1f} mm, "
            f"speed={self.post_return_lateral_speed_hall_s} hall/s"
        )
        if self.post_return_lateral_move_count is None:
            rospy.loginfo(
                "  横移次数策略: 未单独配置，沿用旧逻辑(依据 cycle_count 判断是否继续横移)"
            )
        else:
            rospy.loginfo(f"  横移次数配置: {self.post_return_lateral_move_count}")
        if len(self.post_return_lateral_distances_mm) > 0:
            rospy.loginfo(f"  横移距离列表(mm): {self.post_return_lateral_distances_mm}")
            if self.post_return_lateral_move_count is not None:
                if self.post_return_lateral_move_count > len(self.post_return_lateral_distances_mm):
                    rospy.logwarn(
                        f"横移次数({self.post_return_lateral_move_count})大于距离列表长度"
                        f"({len(self.post_return_lateral_distances_mm)})，超出部分将复用最后一个距离"
                    )
                elif self.post_return_lateral_move_count < len(self.post_return_lateral_distances_mm):
                    rospy.logwarn(
                        f"横移次数({self.post_return_lateral_move_count})小于距离列表长度"
                        f"({len(self.post_return_lateral_distances_mm)})，多余距离不会执行"
                    )
        rospy.loginfo(f"  首次姿态基准: 使用 {self.heading_topic} 的 FAST-LIVO 位姿 yaw")
        rospy.loginfo(
            f"  姿态回正: speed={self.angle_restore_speed_degps:.2f} deg/s, "
            f"tolerance={self.angle_restore_tolerance_deg:.2f} deg"
        )
        rospy.loginfo(
            f"  姿态回正闭环: kp={self.angle_restore_heading_kp:.2f}, "
            f"min_speed={self.angle_restore_min_speed_degps:.2f}, "
            f"max_speed={self.angle_restore_max_speed_degps:.2f}, "
            f"stabilize_count={self.angle_restore_stabilize_count}, "
            f"direct_command_limit={self.angle_restore_command_step_deg:.2f} deg"
        )
        rospy.loginfo(
            f"  角度方向映射: 按现场反馈已反向，负角度->0x01, 正角度/零角度->0x02; "
            f"姿态回正切到角度模式后只发一次 0x0F 位置命令"
        )
        if "/imu" in str(self.heading_topic):
            rospy.logwarn(
                f"  当前 heading_topic={self.heading_topic} 看起来像原始 IMU 话题，"
                "建议改为 /aft_mapped_to_init 以使用 FAST-LIVO 位姿角"
            )
        rospy.loginfo("=" * 70)

    def _load_optional_int_param(self, private_key, global_key):
        if rospy.has_param(private_key):
            raw_value = rospy.get_param(private_key)
        elif rospy.has_param(global_key):
            raw_value = rospy.get_param(global_key)
        else:
            return None

        try:
            return int(raw_value)
        except Exception:
            rospy.logwarn(
                f"整数参数解析失败: private={private_key}, global={global_key}, value={raw_value}"
            )
            return None

    def _load_lateral_distances_param(self, private_key, global_key):
        if rospy.has_param(private_key):
            raw_value = rospy.get_param(private_key)
        elif rospy.has_param(global_key):
            raw_value = rospy.get_param(global_key)
        else:
            return []

        if isinstance(raw_value, (list, tuple)):
            distance_list = []
            for idx, item in enumerate(raw_value):
                try:
                    distance_list.append(float(item))
                except Exception:
                    rospy.logwarn(f"横移距离列表第{idx}项无法解析为数字: {item}")
            return distance_list

        try:
            return [float(raw_value)]
        except Exception:
            rospy.logwarn(
                f"横移距离参数解析失败: private={private_key}, global={global_key}, value={raw_value}"
            )
            return []

    def get_post_return_lateral_distance_mm(self, sequence_id):
        move_index = int(sequence_id)

        if self.post_return_lateral_move_count is not None and move_index > self.post_return_lateral_move_count:
            return None

        if len(self.post_return_lateral_distances_mm) == 0:
            return float(self.post_return_lateral_distance_mm)

        if move_index <= len(self.post_return_lateral_distances_mm):
            return float(self.post_return_lateral_distances_mm[move_index - 1])

        fallback_distance = float(self.post_return_lateral_distances_mm[-1])
        if move_index not in self.logged_lateral_distance_fallback_indices:
            self.logged_lateral_distance_fallback_indices.add(move_index)
            rospy.logwarn(
                f"第{move_index}次横移未配置独立距离，复用最后一个距离 {fallback_distance:.1f} mm"
            )
        return fallback_distance

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
            filter_ids = {int(self.can_id), int(self.harvest_done_can_id)}
            filters = [
                {"can_id": can_id, "can_mask": 0x7FF, "extended": False}
                for can_id in sorted(filter_ids)
            ]
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

    def _append_harvest_frame(self, msg):
        with self.harvest_rx_lock:
            recv_time = time.time()
            self.harvest_last_rx_wall_time = recv_time
            self.harvest_rx_queue.append({"msg": msg, "recv_time": recv_time})

    def _pop_harvest_frame(self):
        with self.harvest_rx_lock:
            if self.harvest_rx_queue:
                return self.harvest_rx_queue.popleft()
        return None

    def _clear_harvest_rx_queue(self):
        with self.harvest_rx_lock:
            self.harvest_rx_queue.clear()

    def _get_harvest_last_rx_wall_time(self):
        with self.harvest_rx_lock:
            return float(self.harvest_last_rx_wall_time)

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

            if arbitration_id == self.can_id:
                self._append_ctrl_frame(msg)
            elif arbitration_id == self.harvest_done_can_id:
                self._append_harvest_frame(msg)

    @staticmethod
    def _quaternion_to_yaw_deg(quat):
        x = float(quat.x)
        y = float(quat.y)
        z = float(quat.z)
        w = float(quat.w)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def heading_callback(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            stamp = rospy.Time.now().to_sec()

        pose = msg.pose.pose
        yaw_deg = self._normalize_angle_deg(self._quaternion_to_yaw_deg(pose.orientation))
        now_wall = time.time()
        position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )

        with self.heading_lock:
            self.heading_last_ros_stamp = stamp
            self.heading_last_recv_wall_time = now_wall
            self.heading_last_yaw_deg = yaw_deg
            self.heading_last_position = position

        self._try_capture_initial_heading_reference(yaw_deg, position, now_wall)

    def _format_position_text(self, position):
        if position is None:
            return "None"
        return f"({position[0]:+.2f}, {position[1]:+.2f}, {position[2]:+.2f})"

    def _try_capture_initial_heading_reference(self, yaw_deg, position, capture_wall_time):
        if not self.capture_initial_heading_before_first_cycle:
            return False
        if yaw_deg is None:
            return False

        captured = False
        normalized_yaw_deg = self._normalize_angle_deg(float(yaw_deg))
        with self.heading_lock:
            if self.heading_reference_deg is None:
                self.heading_reference_deg = normalized_yaw_deg
                self.heading_reference_source = "startup_fast"
                self.heading_reference_capture_wall_time = float(capture_wall_time)
                captured = True

        if not captured:
            return False

        rospy.loginfo(
            f"启动阶段已锁存首次 FAST 基准角: yaw={normalized_yaw_deg:+.2f} deg, "
            f"source={self.heading_topic}, pos={self._format_position_text(position)}"
        )
        rospy.loginfo(
            f"后续所有轮次都会回到这个启动前置基准角 {normalized_yaw_deg:+.2f} deg"
        )
        return True

    def send_can_message(self, data, arbitration_id=None):
        if self.can_bus is None or can is None:
            if not self.running or rospy.is_shutdown():
                return True
            can_id = self.can_id if arbitration_id is None else int(arbitration_id)
            rospy.loginfo(f"[模拟CAN][0x{can_id:03X}] {self._format_can_data(data)}")
            return True

        try:
            msg = can.Message(
                arbitration_id=self.can_id if arbitration_id is None else int(arbitration_id),
                data=data,
                is_extended_id=False,
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            if "file descriptor cannot be a negative integer" in str(e):
                self.can_bus = None
            if not self.running or rospy.is_shutdown():
                rospy.logwarn(f"CAN 发送失败(节点退出中): {e}")
            else:
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

                raw_speed = (msg.data[3] << 8) | msg.data[4]
                return {
                    "angle": (msg.data[1] << 8) | msg.data[2],
                    "speed": raw_speed,
                    "speed_signed": self._uint16_to_int16(raw_speed),
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
    def _uint16_to_int16(value):
        value = int(value) & 0xFFFF
        if value & 0x8000:
            return value - 0x10000
        return value

    def _get_status_speed_signed(self, status):
        speed = int(status.get("speed", 0))
        if "speed_signed" in status:
            return int(status["speed_signed"])
        return self._uint16_to_int16(speed)

    def _get_status_speed_abs(self, status):
        return abs(self._get_status_speed_signed(status))

    def _is_status_speed_zero(self, status):
        speed = int(status.get("speed", 0))
        if speed == 0:
            return True
        return self._get_status_speed_abs(status) <= int(self.stop_speed_zero_deadband)

    @staticmethod
    def _mm_to_hall(distance_mm):
        return int(round(abs(distance_mm) / DEFAULT_WHEEL_HALL_TO_MM))

    def _deg_to_angle_cnt(self, angle_deg):
        cnt = int(round(abs(float(angle_deg)) * max(abs(self.angle_deg_to_cnt), 1e-6)))
        return max(0, min(0xFFFF, cnt))

    @staticmethod
    def _normalize_angle_deg(angle_deg):
        angle_deg = float(angle_deg)
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    @staticmethod
    def _format_can_data(data):
        return " ".join(f"{int(b):02X}" for b in data)

    def _compute_angle_restore_speed_degps(self, delta_deg):
        speed_degps = self.angle_restore_heading_kp * abs(float(delta_deg))
        speed_degps = max(float(self.angle_restore_min_speed_degps), speed_degps)
        speed_degps = min(float(self.angle_restore_max_speed_degps), speed_degps)
        return speed_degps

    def _compute_angle_restore_step_deg(self, delta_deg):
        step_cap_deg = max(float(self.angle_restore_tolerance_deg), float(self.angle_restore_command_step_deg))
        delta_deg = float(delta_deg)
        return max(-step_cap_deg, min(step_cap_deg, delta_deg))

    def wait_for_heading_snapshot(self, timeout=None):
        snapshot = self.get_heading_yaw_deg()
        if timeout is None or float(timeout) <= 0:
            while snapshot is None and not rospy.is_shutdown():
                time.sleep(0.02)
                snapshot = self.get_heading_yaw_deg()
            return snapshot

        deadline = time.time() + max(0.0, float(timeout))
        while snapshot is None and time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.02)
            snapshot = self.get_heading_yaw_deg()
        return snapshot

    @staticmethod
    def _planar_distance_mm(position_a, position_b):
        if position_a is None or position_b is None:
            return None

        dx = float(position_a[0]) - float(position_b[0])
        dy = float(position_a[1]) - float(position_b[1])
        return math.hypot(dx, dy) * 1000.0

    @staticmethod
    def _normalize_planar_vector(dx, dy):
        norm = math.hypot(float(dx), float(dy))
        if norm <= 1e-9:
            return None
        return float(dx) / norm, float(dy) / norm

    def _build_fast_return_axis(self):
        if self.start_fast_position is None:
            return None

        if self.forward_end_fast_position is not None:
            dx = float(self.forward_end_fast_position[0]) - float(self.start_fast_position[0])
            dy = float(self.forward_end_fast_position[1]) - float(self.start_fast_position[1])
            axis = self._normalize_planar_vector(dx, dy)
            if axis is not None:
                return {
                    "ux": axis[0],
                    "uy": axis[1],
                    "source": "forward_track",
                    "norm_mm": math.hypot(dx, dy) * 1000.0,
                }

        yaw_deg = self.start_fast_yaw_deg
        if yaw_deg is None:
            yaw_deg = self.current_cycle_pre_uprail_angle_deg
        if yaw_deg is None:
            yaw_deg = self.heading_reference_deg
        if yaw_deg is None:
            return None

        yaw_rad = math.radians(float(yaw_deg))
        return {
            "ux": math.cos(yaw_rad),
            "uy": math.sin(yaw_rad),
            "source": "start_yaw",
            "norm_mm": None,
        }

    def get_signed_fast_offset_mm(self):
        if self.start_fast_position is None:
            return None, None, None

        heading_snapshot = self.get_heading_yaw_deg()
        if heading_snapshot is None or heading_snapshot.get("position") is None:
            return None, None, None

        axis = self._build_fast_return_axis()
        if axis is None:
            return None, heading_snapshot, None

        current_position = heading_snapshot["position"]
        dx = float(current_position[0]) - float(self.start_fast_position[0])
        dy = float(current_position[1]) - float(self.start_fast_position[1])
        signed_offset_mm = (dx * float(axis["ux"]) + dy * float(axis["uy"])) * 1000.0
        cross_track_mm = abs((-dy * float(axis["ux"]) + dx * float(axis["uy"])) * 1000.0)
        axis_snapshot = dict(axis)
        axis_snapshot["cross_track_mm"] = cross_track_mm
        return signed_offset_mm, heading_snapshot, axis_snapshot

    @staticmethod
    def _format_return_x_error_text(axis):
        if not isinstance(axis, dict) or "x_error_mm" not in axis:
            return ""
        try:
            return (
                f", current_x={float(axis['current_x_m']):+.3f} m, "
                f"target_x={float(axis['target_x_m']):+.3f} m, "
                f"x_error={float(axis['x_error_mm']):+.1f} mm"
            )
        except Exception:
            return ""

    def _wheel_value_to_mm(self, value, unit):
        if value is None:
            return None
        if unit == "mm":
            return float(value)
        if unit == "hall":
            hall_to_mm = self.wheel_hall_to_mm if self.wheel_hall_to_mm > 0 else DEFAULT_WHEEL_HALL_TO_MM
            return float(value) * hall_to_mm
        return None

    def get_distance_to_fast_start_mm(self):
        if self.start_fast_position is None:
            return None, None

        heading_snapshot = self.get_heading_yaw_deg()
        if heading_snapshot is None:
            return None, None

        current_position = heading_snapshot["position"]
        distance_mm = self._planar_distance_mm(current_position, self.start_fast_position)
        return distance_mm, heading_snapshot

    def get_heading_yaw_deg(self):
        with self.heading_lock:
            last_recv_wall_time = self.heading_last_recv_wall_time
            yaw_deg = self.heading_last_yaw_deg
            last_ros_stamp = self.heading_last_ros_stamp
            position = self.heading_last_position

        if last_recv_wall_time <= 0 or yaw_deg is None:
            return None

        age_sec = time.time() - last_recv_wall_time
        if age_sec > self.heading_timeout_sec:
            rospy.logwarn(
                f"FAST-LIVO 位姿数据超时: topic={self.heading_topic}, "
                f"age={age_sec:.2f}s, timeout={self.heading_timeout_sec:.2f}s"
            )
            return None

        return {
            "yaw_deg": float(yaw_deg),
            "stamp": last_ros_stamp,
            "age_sec": age_sec,
            "position": position,
        }

    def log_heading_snapshot(self, prefix):
        heading_snapshot = self.get_heading_yaw_deg()
        ref_text = "None" if self.heading_reference_deg is None else f"{float(self.heading_reference_deg):+.2f} deg"
        cycle_text = (
            "None" if self.current_cycle_pre_uprail_angle_deg is None
            else f"{float(self.current_cycle_pre_uprail_angle_deg):+.2f} deg"
        )

        if heading_snapshot is None:
            rospy.logwarn(
                f"{prefix}: 无法读取当前姿态角 | 首次基准角={ref_text}, 本轮起始角={cycle_text}"
            )
            return None

        current_angle_deg = float(heading_snapshot["yaw_deg"])
        delta_to_ref = None if self.heading_reference_deg is None else (
            self._normalize_angle_deg(float(self.heading_reference_deg) - current_angle_deg)
        )
        delta_text = "None" if delta_to_ref is None else f"{delta_to_ref:+.2f} deg"
        position = heading_snapshot["position"]
        position_text = (
            "None" if position is None
            else f"({position[0]:+.2f}, {position[1]:+.2f}, {position[2]:+.2f})"
        )
        rospy.loginfo(
            f"{prefix}: 首次基准角={ref_text}, 本轮起始角={cycle_text}, "
            f"当前姿态角={current_angle_deg:+.2f} deg, 与首次基准角差值={delta_text}, "
            f"source={self.heading_topic}, pos={position_text}, age={heading_snapshot['age_sec']:.3f}s"
        )
        return {
            "current_angle_deg": current_angle_deg,
            "delta_to_reference_deg": delta_to_ref,
            "reference_angle_deg": None if self.heading_reference_deg is None else float(self.heading_reference_deg),
            "cycle_start_angle_deg": None if self.current_cycle_pre_uprail_angle_deg is None else float(self.current_cycle_pre_uprail_angle_deg),
        }

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

    def send_harvest_status(self, ready):
        payload = self.harvest_ready_payload if ready else self.harvest_busy_payload
        state_text = "到位允许采摘" if ready else "未到位/移动中"
        rospy.loginfo(
            f"发送采摘状态: {state_text}, can_id=0x{self.harvest_status_can_id:03X}, "
            f"data={self._format_can_data(payload)}"
        )
        sent = self.send_can_message(payload, arbitration_id=self.harvest_status_can_id)
        if sent:
            if ready:
                self.harvest_ready_sent_wall_time = time.time()
            else:
                self.harvest_ready_sent_wall_time = 0.0
        return sent

    def wait_for_harvest_complete(self, phase_name):
        if self.can_bus is None or can is None:
            rospy.loginfo(f"{phase_name}: [模拟CAN] 默认采摘完成，继续执行")
            return True

        timeout_sec = float(self.harvest_wait_timeout_sec)
        deadline = None if timeout_sec <= 0.0 else time.time() + timeout_sec
        accept_after = max(
            float(self.harvest_ready_sent_wall_time) + float(self.harvest_arm_delay_sec),
            float(self.harvest_done_ignore_until),
        )
        rospy.loginfo(
            f"{phase_name}: 等待采摘完成信号 can_id=0x{self.harvest_done_can_id:03X}, "
            f"data={self._format_can_data(self.harvest_done_payload)}, "
            f"timeout={'disabled' if deadline is None else f'{timeout_sec:.1f}s'}, "
            f"accept_after={accept_after:.3f}"
        )

        while self.running and not rospy.is_shutdown():
            entry = self._pop_harvest_frame()
            if entry is None:
                if deadline is not None and time.time() >= deadline:
                    rospy.logerr(f"{phase_name}: 等待采摘完成信号超时")
                    return False
                time.sleep(0.02)
                continue

            msg = entry["msg"]
            recv_time = float(entry["recv_time"])
            if recv_time < accept_after:
                rospy.loginfo(
                    f"{phase_name}: 忽略重复/过早采摘信号 can_id=0x{int(msg.arbitration_id):03X}, "
                    f"recv_time={recv_time:.3f}, accept_after={accept_after:.3f}, "
                    f"data={self._format_can_data(msg.data[:8])}"
                )
                continue

            payload = bytes(msg.data[:8])
            if payload == self.harvest_done_payload:
                self.harvest_done_ignore_until = time.time() + float(self.harvest_duplicate_suppress_sec)
                self._clear_harvest_rx_queue()
                rospy.loginfo(
                    f"{phase_name}: 收到采摘完成信号 can_id=0x{int(msg.arbitration_id):03X}, "
                    f"data={self._format_can_data(payload)}"
                )
                return True

            rospy.logwarn(
                f"{phase_name}: 忽略非预期采摘信号 can_id=0x{int(msg.arbitration_id):03X}, "
                f"data={self._format_can_data(payload)}"
            )

        return False

    def wait_for_harvest_rearm_window(self, phase_name):
        if self.can_bus is None or can is None:
            return True

        quiet_sec = float(self.harvest_rearm_quiet_sec)
        self.harvest_done_ignore_until = 0.0
        if quiet_sec <= 0.0:
            self._clear_harvest_rx_queue()
            return True

        wait_logged = False
        while self.running and not rospy.is_shutdown():
            last_rx_wall_time = self._get_harvest_last_rx_wall_time()
            if last_rx_wall_time <= 0.0:
                self._clear_harvest_rx_queue()
                return True

            quiet_elapsed = time.time() - last_rx_wall_time
            if quiet_elapsed >= quiet_sec:
                self._clear_harvest_rx_queue()
                rospy.loginfo(
                    f"{phase_name}: 021 采摘完成信号已静默 {quiet_elapsed:.2f}s，"
                    "重新允许等待下一次完成信号"
                )
                return True

            if not wait_logged:
                rospy.logwarn(
                    f"{phase_name}: 检测到 021 仍在连续上报，需静默 {quiet_sec:.2f}s "
                    "后才重新允许下一次前进"
                )
                wait_logged = True

            rospy.loginfo_throttle(
                0.5,
                f"{phase_name}: 等待 021 静默窗口 quiet_elapsed={quiet_elapsed:.2f}s, "
                f"required={quiet_sec:.2f}s"
            )
            time.sleep(0.02)

        return False

    def send_angle_position_command(self, angle_deg_signed, speed_degps):
        angle_abs = abs(float(angle_deg_signed))
        speed_degps = max(0.1, abs(float(speed_degps)))
        direction_byte = 0x02 if angle_deg_signed >= 0 else 0x01
        position_cnt = self._deg_to_angle_cnt(angle_abs)
        speed_cnt = self._deg_to_angle_cnt(speed_degps)

        data = bytearray(8)
        data[0] = 0x0F
        data[1] = (position_cnt >> 8) & 0xFF
        data[2] = position_cnt & 0xFF
        data[3] = (speed_cnt >> 8) & 0xFF
        data[4] = speed_cnt & 0xFF
        data[5] = direction_byte
        data[6] = 0x00
        data[7] = 0x00

        rospy.loginfo(
            f"发送角度位置命令: angle={angle_deg_signed:+.2f} deg, "
            f"speed={speed_degps:.2f} deg/s, direction=0x{direction_byte:02X}"
        )
        rospy.loginfo(f"[0x0F] 数据: {' '.join(f'{int(b):02X}' for b in data)}")
        return self.send_can_message(data)

    def send_angle_hold_command(self):
        hold_speed_degps = max(0.1, min(float(self.angle_restore_min_speed_degps), 0.6))
        rospy.loginfo(
            f"发送角度保持命令: angle=+0.00 deg, speed={hold_speed_degps:.2f} deg/s"
        )
        return self.send_angle_position_command(0.0, hold_speed_degps)

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
            speed_signed = self._get_status_speed_signed(status)
            speed_is_zero = self._is_status_speed_zero(status)
            if speed_signed == speed:
                deadband_text = ""
                if speed_is_zero and speed != 0:
                    deadband_text = f" (deadband={self.stop_speed_zero_deadband}, 按0计)"
                rospy.loginfo_throttle(0.5, f"{phase_name}: 当前反馈速度={speed}{deadband_text}")
            else:
                deadband_text = ""
                if speed_is_zero and speed != 0:
                    deadband_text = f", deadband={self.stop_speed_zero_deadband}, 按0计"
                rospy.loginfo_throttle(
                    0.5,
                    f"{phase_name}: 当前反馈速度={speed_signed} (raw={speed}{deadband_text})"
                )

            if speed_is_zero:
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
            speed_abs = self._get_status_speed_abs(status)
            motion_state = status.get("motion_state")
            if (
                speed_abs > self.stop_speed_zero_deadband or
                not status["arrived"] or
                (initial_motion_state is not None and motion_state != initial_motion_state)
            ):
                motion_started = True
                rospy.loginfo(
                    f"{phase_name}: 检测到动作开始, speed={self._get_status_speed_signed(status)} "
                    f"(raw={speed}), "
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
            speed_signed = self._get_status_speed_signed(status)
            speed_abs = abs(speed_signed)
            speed_is_zero = self._is_status_speed_zero(status)
            arrived = status["arrived"]
            motion_state = status.get("motion_state")
            rospy.loginfo_throttle(
                0.5,
                f"{phase_name}: mode=0x{status['mode']:02X}, speed={speed_signed} (raw={speed}), "
                f"arrived={arrived}, motion_state=0x{motion_state:02X}, "
                f"started={motion_started}"
            )

            if (
                speed_abs > self.stop_speed_zero_deadband or
                not arrived or
                (initial_motion_state is not None and motion_state != initial_motion_state)
            ):
                motion_started = True
                zero_count = 0
            elif speed_is_zero and arrived and time.time() - start_time >= min_wait:
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

    def wait_angle_motion_complete(self, phase_name, angle_deg, speed_degps, timeout=None):
        speed_degps = max(0.1, abs(float(speed_degps)))
        expected_duration = abs(float(angle_deg)) / speed_degps + 1.5
        if timeout is None:
            timeout = max(self.angle_restore_timeout_sec, expected_duration + 2.0)

        rospy.loginfo(
            f"{phase_name}: 目标角度={float(angle_deg):+.2f} deg, "
            f"速度={speed_degps:.2f} deg/s, 预计时长={expected_duration:.2f} s"
        )
        return self.wait_position_motion_complete(
            phase_name=phase_name,
            timeout=timeout,
            expected_duration=expected_duration,
            allow_implicit_complete=True,
        )

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
        self.start_odom_hall = None
        self.last_odom_hall = None
        self.start_odom_mm = None
        self.last_odom_mm = None
        self.start_fast_position = None
        self.start_fast_yaw_deg = None
        self.forward_end_fast_position = None
        self.forward_end_fast_yaw_deg = None

        heading_snapshot = self.wait_for_heading_snapshot(timeout=0.0)
        if heading_snapshot is not None and heading_snapshot.get("position") is not None:
            self.start_fast_position = tuple(heading_snapshot["position"])
            self.start_fast_yaw_deg = float(heading_snapshot["yaw_deg"])
            rospy.loginfo(
                f"记录起点 FAST 位姿: "
                f"({self.start_fast_position[0]:+.3f}, {self.start_fast_position[1]:+.3f}, {self.start_fast_position[2]:+.3f}), "
                f"yaw={self.start_fast_yaw_deg:+.2f} deg"
            )
        else:
            rospy.logwarn(f"记录起点 FAST 位姿失败: 无法从 {self.heading_topic} 获取有效位置")

    def get_relative_distance_mm(self):
        if self.start_odom_mm is None:
            return None

        current_odom = self.query_wheel_odometry_mm()
        self.last_odom_mm = current_odom
        if current_odom is None:
            return None

        return float(current_odom - self.start_odom_mm)

    def record_forward_end_odometry(self):
        heading_snapshot = self.get_heading_yaw_deg()
        if heading_snapshot is not None and heading_snapshot.get("position") is not None:
            self.forward_end_fast_position = tuple(heading_snapshot["position"])
            self.forward_end_fast_yaw_deg = float(heading_snapshot["yaw_deg"])
            forward_distance_mm = self._planar_distance_mm(self.forward_end_fast_position, self.start_fast_position)
            if forward_distance_mm is None:
                rospy.loginfo(
                    f"记录前进终点 FAST 位姿: "
                    f"({self.forward_end_fast_position[0]:+.3f}, {self.forward_end_fast_position[1]:+.3f}, {self.forward_end_fast_position[2]:+.3f}), "
                    f"yaw={self.forward_end_fast_yaw_deg:+.2f} deg"
                )
            else:
                axis = self._build_fast_return_axis()
                axis_text = "unknown"
                if axis is not None:
                    axis_text = axis["source"]
                rospy.loginfo(
                    f"记录前进终点 FAST 位姿: "
                    f"({self.forward_end_fast_position[0]:+.3f}, {self.forward_end_fast_position[1]:+.3f}, {self.forward_end_fast_position[2]:+.3f}), "
                    f"与起点平面距离={forward_distance_mm:.1f} mm, "
                    f"回零轴来源={axis_text}"
                )
        else:
            self.forward_end_fast_position = None
            self.forward_end_fast_yaw_deg = None
            rospy.logwarn(f"记录前进终点 FAST 位姿失败: 当前无有效 {self.heading_topic} 位姿")

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

    def load_external_heading_reference(self):
        if not self.use_external_heading_reference:
            return None

        if not rospy.has_param(self.external_heading_reference_param):
            return None

        payload = rospy.get_param(self.external_heading_reference_param)
        yaw_value = None
        stamp_value = None

        if isinstance(payload, dict):
            yaw_value = payload.get("yaw_deg")
            stamp_value = payload.get("stamp")
        elif isinstance(payload, (int, float)):
            yaw_value = payload
        else:
            rospy.logwarn(
                f"外部基准角参数格式不支持: param={self.external_heading_reference_param}, "
                f"type={type(payload).__name__}"
            )
            return None

        try:
            yaw_deg = self._normalize_angle_deg(float(yaw_value))
        except Exception:
            rospy.logwarn(
                f"外部基准角参数无效: param={self.external_heading_reference_param}, value={yaw_value}"
            )
            return None

        age_sec = None
        if stamp_value is not None:
            try:
                stamp_sec = float(stamp_value)
                age_sec = max(0.0, time.time() - stamp_sec)
                if (
                    self.external_heading_reference_max_age_sec > 0
                    and age_sec > self.external_heading_reference_max_age_sec
                ):
                    rospy.logwarn(
                        f"外部基准角参数过期: age={age_sec:.1f}s > "
                        f"max_age={self.external_heading_reference_max_age_sec:.1f}s, "
                        f"param={self.external_heading_reference_param}"
                    )
                    return None
            except Exception:
                age_sec = None

        return {
            "yaw_deg": yaw_deg,
            "age_sec": age_sec,
            "raw": payload,
        }

    def ensure_heading_reference(self):
        heading_snapshot = self.get_heading_yaw_deg()
        if heading_snapshot is None:
            rospy.logwarn(f"无法读取 {self.heading_topic}，不能建立/检查姿态基准")
            self.current_cycle_pre_uprail_angle_deg = None
            return False

        current_angle_deg = float(heading_snapshot["yaw_deg"])
        self.current_cycle_pre_uprail_angle_deg = current_angle_deg

        if self.heading_reference_deg is None:
            external_reference = self.load_external_heading_reference()
            if external_reference is not None:
                self.heading_reference_deg = float(external_reference["yaw_deg"])
                self.heading_reference_source = "external_param"
                self.heading_reference_capture_wall_time = time.time()
                age_text = (
                    "unknown"
                    if external_reference["age_sec"] is None
                    else f"{external_reference['age_sec']:.3f}s"
                )
                rospy.loginfo(
                    f"采用外部基准角: yaw={self.heading_reference_deg:+.2f} deg, "
                    f"param={self.external_heading_reference_param}, age={age_text}"
                )
            else:
                self.heading_reference_deg = current_angle_deg
                self.heading_reference_source = "sequence_fast_fallback"
                self.heading_reference_capture_wall_time = time.time()
                position = heading_snapshot["position"]
                rospy.loginfo(
                    f"记录首次 SLAM 基准角: yaw={self.heading_reference_deg:+.2f} deg, "
                    f"topic={self.heading_topic}, pos={self._format_position_text(position)}, "
                    f"age={heading_snapshot['age_sec']:.3f}s"
                )
            rospy.loginfo(
                f"当前程序运行期间后续所有轮次，都会回到这个首次 SLAM 基准角 "
                f"{self.heading_reference_deg:+.2f} deg"
            )
        else:
            delta_deg = float(self.heading_reference_deg) - current_angle_deg
            source_text = self.heading_reference_source or "unknown"
            rospy.loginfo(
                f"沿用首次 SLAM 基准角: ref={self.heading_reference_deg:+.2f} deg, "
                f"current={current_angle_deg:+.2f} deg, delta={delta_deg:+.2f} deg, "
                f"source={source_text}"
            )

        self.log_heading_snapshot("姿态基准检查")

        return True

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

    def execute_post_return_lateral_move(self, sequence_id):
        distance_mm = self.get_post_return_lateral_distance_mm(sequence_id)
        if distance_mm is None:
            rospy.loginfo(f"第 {sequence_id} 轮不在横移次数计划内，跳过横移")
            return True

        distance_mm = float(distance_mm)
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

    def execute_post_return_angle_restore(self):
        if self.heading_reference_deg is None:
            rospy.logwarn("姿态回正失败: 尚未记录首次 SLAM 基准角")
            return False

        target_angle_deg = float(self.heading_reference_deg)
        if not self.switch_mode_if_needed(self.MODE_ANGLE):
            rospy.logwarn("姿态回正失败: 无法切换到角度模式")
            return False

        self.log_heading_snapshot("姿态回正开始前")
        rospy.loginfo(
            "姿态回正: 切到角度模式确认完成后，只发送一次 0x0F 位置命令；"
            "后续仅监看 FAST-LIVO yaw 是否回到容差内，不再重复补发"
        )

        heading_snapshot = self.get_heading_yaw_deg()
        if heading_snapshot is None:
            rospy.logwarn(f"姿态回正: 无法读取当前 {self.heading_topic} 角度")
            return False

        current_angle_deg = float(heading_snapshot["yaw_deg"])
        delta_deg = self._normalize_angle_deg(target_angle_deg - current_angle_deg)
        abs_delta_deg = abs(delta_deg)

        rospy.loginfo(
            f"姿态回正单次发令前: current={current_angle_deg:+.2f} deg, "
            f"target={target_angle_deg:+.2f} deg, delta={delta_deg:+.2f} deg, "
            f"source={self.heading_topic}, age={heading_snapshot['age_sec']:.3f}s"
        )

        if abs_delta_deg <= self.angle_restore_tolerance_deg:
            rospy.loginfo(f"姿态回正: 当前已在容差内，无需下发角度命令 (delta={delta_deg:+.2f} deg)")
            self.log_heading_snapshot("姿态回正完成后")
            return True

        command_delta_deg = self._compute_angle_restore_step_deg(delta_deg)
        command_speed_degps = self._compute_angle_restore_speed_degps(delta_deg)
        direction_byte = 0x02 if command_delta_deg >= 0 else 0x01
        command_mode_text = (
            "直接使用当前差值"
            if abs(command_delta_deg - delta_deg) <= 1e-6
            else f"角度过大，单次限幅到 {command_delta_deg:+.2f} deg"
        )
        rospy.loginfo(
            f"姿态回正单次发令: current={current_angle_deg:+.2f} deg, "
            f"target={target_angle_deg:+.2f} deg, delta={delta_deg:+.2f} deg "
            f"-> cmd_angle={command_delta_deg:+.2f} deg, dir=0x{direction_byte:02X}, "
            f"cmd_speed={command_speed_degps:.2f} deg/s, {command_mode_text}"
        )
        if not self.send_angle_position_command(command_delta_deg, command_speed_degps):
            return False

        stable_count = 0
        loop_rate = rospy.Rate(max(2.0, float(self.angle_restore_poll_hz)))
        start_time = time.time()

        while time.time() - start_time < self.angle_restore_timeout_sec and not rospy.is_shutdown():
            heading_snapshot = self.get_heading_yaw_deg()
            if heading_snapshot is None:
                rospy.logwarn(f"姿态回正: 无法读取当前 {self.heading_topic} 角度")
                return False

            current_angle_deg = float(heading_snapshot["yaw_deg"])
            delta_deg = self._normalize_angle_deg(target_angle_deg - current_angle_deg)
            abs_delta_deg = abs(delta_deg)

            rospy.loginfo_throttle(
                0.5,
                f"姿态回正监看: current={current_angle_deg:+.2f} deg, "
                f"target={target_angle_deg:+.2f} deg, delta={delta_deg:+.2f} deg, "
                f"stable={stable_count}/{self.angle_restore_stabilize_count}, command_sent=1, "
                f"source={self.heading_topic}, age={heading_snapshot['age_sec']:.3f}s"
            )

            if abs_delta_deg <= self.angle_restore_tolerance_deg:
                stable_count += 1
                if stable_count >= self.angle_restore_stabilize_count:
                    rospy.loginfo(
                        f"姿态回正完成: 已连续 {stable_count} 次回到首次 SLAM 基准角附近 "
                        f"(误差={delta_deg:+.2f} deg)"
                    )
                    self.log_heading_snapshot("姿态回正完成后")
                    return True
            else:
                stable_count = 0

            loop_rate.sleep()

        final_snapshot = self.get_heading_yaw_deg()
        if final_snapshot is not None:
            final_angle_deg = float(final_snapshot["yaw_deg"])
            final_delta_deg = self._normalize_angle_deg(target_angle_deg - final_angle_deg)
            rospy.logwarn(
                f"姿态回正失败: 最终 current={final_angle_deg:+.2f} deg, "
                f"target={target_angle_deg:+.2f} deg, delta={final_delta_deg:+.2f} deg, "
                f"stable={stable_count}/{self.angle_restore_stabilize_count}, "
                f"commands=1"
            )
        else:
            rospy.logwarn(f"姿态回正失败: 无法读取最终 {self.heading_topic} 角度")
        return False

    def should_run_post_return_lateral_move(self, sequence_id):
        if self.post_return_lateral_move_count is not None:
            return int(sequence_id) <= self.post_return_lateral_move_count

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
        rospy.loginfo("阶段1: 切四轮转向模式后按 FAST/SLAM 距离闭环分段上轨")
        rospy.loginfo("=" * 60)

        self.record_start_odometry()
        if self.start_fast_position is None:
            rospy.logerr("前进阶段失败: 未记录到有效起点 FAST 位姿")
            return False

        total_target_mm = max(0.0, float(self.forward_distance_mm))
        if total_target_mm <= 0.0:
            rospy.loginfo("前进目标距离为 0，跳过分段上轨")
            return True

        if not self.send_harvest_status(False):
            return False

        current_distance_mm = 0.0
        segment_index = 0

        while self.running and not rospy.is_shutdown():
            fast_distance_mm, _ = self.get_distance_to_fast_start_mm()
            if fast_distance_mm is not None:
                current_distance_mm = max(0.0, abs(float(fast_distance_mm)))

            remaining_distance_mm = max(0.0, total_target_mm - current_distance_mm)
            if remaining_distance_mm <= 0.0:
                rospy.loginfo(
                    f"前进 FAST 距离达到总目标: current_mm={current_distance_mm:.1f}, "
                    f"target_mm={total_target_mm:.1f}"
                )
                break

            if remaining_distance_mm <= self.forward_pick_direct_finish_threshold_mm:
                segment_distance_mm = remaining_distance_mm
            else:
                segment_distance_mm = min(self.forward_pick_step_mm, remaining_distance_mm)

            absolute_target_mm = current_distance_mm + segment_distance_mm
            segment_index += 1
            phase_name = f"前进分段{segment_index}"
            rospy.loginfo(
                f"{phase_name}: current_mm={current_distance_mm:.1f}, remaining_mm={remaining_distance_mm:.1f}, "
                f"segment_mm={segment_distance_mm:.1f}, absolute_target_mm={absolute_target_mm:.1f}, "
                f"total_target_mm={total_target_mm:.1f}"
            )

            if not self.send_speed_mode_command(self.forward_linear_speed_mm_s, self.angular_speed_cmd):
                return False

            last_distance_mm = current_distance_mm
            while self.running and not rospy.is_shutdown():
                fast_distance_mm, _ = self.get_distance_to_fast_start_mm()

                if fast_distance_mm is not None:
                    last_distance_mm = max(0.0, abs(float(fast_distance_mm)))
                    rospy.loginfo_throttle(
                        0.5,
                        f"{phase_name}: fast_dist_mm={last_distance_mm:.1f}, "
                        f"segment_target_mm={absolute_target_mm:.1f}, total_target_mm={total_target_mm:.1f}"
                    )
                    if last_distance_mm >= absolute_target_mm:
                        rospy.loginfo(
                            f"{phase_name}: 到达分段停车点 fast_dist_mm={last_distance_mm:.1f}, "
                            f"segment_target_mm={absolute_target_mm:.1f}"
                        )
                        break
                else:
                    rospy.logwarn_throttle(
                        1.0,
                        f"{phase_name}: 暂时无法读取 FAST 距离，继续等待 {self.heading_topic}"
                    )

                time.sleep(0.02)

            self.stop_speed_motion()
            self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
            time.sleep(self.stop_settle_sec)

            fast_distance_mm, _ = self.get_distance_to_fast_start_mm()
            if fast_distance_mm is not None:
                current_distance_mm = max(0.0, abs(float(fast_distance_mm)))
            else:
                current_distance_mm = last_distance_mm

            rospy.loginfo(
                f"{phase_name}: 停车完成 actual_fast_dist_mm={current_distance_mm:.1f}, "
                f"target_mm={total_target_mm:.1f}"
            )

            if not self.wait_for_harvest_rearm_window(phase_name):
                return False
            if not self.send_harvest_status(True):
                return False
            if not self.wait_for_harvest_complete(phase_name):
                return False

            if current_distance_mm >= total_target_mm:
                rospy.loginfo(
                    f"{phase_name}: 已完成最终前进目标 current_mm={current_distance_mm:.1f}, "
                    f"target_mm={total_target_mm:.1f}"
                )
                break

            if not self.send_harvest_status(False):
                return False

        if not self.send_harvest_status(False):
            return False

        self.record_forward_end_odometry()
        return True

    def _run_reverse_phase_time_fallback(self):
        rospy.logwarn("FAST 回零不可用，退回纯时间兜底倒车")
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
        return True

    def run_reverse_phase_wheel_primary(self):
        plan = self.get_return_plan()
        if plan is None:
            return self._run_reverse_phase_time_fallback()

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

    def run_reverse_phase_fast_primary(self):
        if self.start_fast_position is None:
            rospy.logwarn("FAST 主回零不可用: 未记录起点 FAST 位姿")
            return None

        initial_signed_mm, _, axis = self.get_signed_fast_offset_mm()
        if initial_signed_mm is None:
            rospy.logwarn(f"FAST 主回零不可用: 无法读取 {self.heading_topic} 当前有符号位移")
            return None

        initial_signed_mm = float(initial_signed_mm)
        tolerance_mm = max(1.0, float(self.return_fast_tolerance_mm))
        reverse_speed_abs = max(1, abs(int(self.reverse_linear_speed_mm_s)))
        correction_speed_abs = max(1, abs(int(self.return_correction_speed_mm_s)))

        rospy.loginfo(
            f"FAST 回零计划: initial_signed={initial_signed_mm:+.1f} mm, "
            f"initial_abs={abs(initial_signed_mm):.1f} mm, tol={tolerance_mm:.1f} mm, "
            f"reverse_speed={-reverse_speed_abs} mm/s, correction_speed={correction_speed_abs} mm/s, "
            f"slowdown_dist={self.return_fast_slowdown_distance_mm:.1f} mm, "
            f"axis={axis['source'] if axis is not None else 'unknown'}"
            f"{self._format_return_x_error_text(axis)}"
        )

        if abs(initial_signed_mm) <= tolerance_mm:
            rospy.loginfo("FAST 回零: 起始已在容差内，跳过倒车")
            return True

        best_abs_signed_mm = abs(initial_signed_mm)
        phase_index = 0

        while self.running and not rospy.is_shutdown():
            current_signed_mm, _, axis = self.get_signed_fast_offset_mm()
            if current_signed_mm is None:
                rospy.logwarn_throttle(
                    1.0,
                    f"FAST 回零: 暂时读不到 {self.heading_topic} 的有符号位移，继续等待..."
                )
                time.sleep(0.02)
                continue

            current_signed_mm = float(current_signed_mm)
            current_abs_mm = abs(current_signed_mm)
            best_abs_signed_mm = min(best_abs_signed_mm, current_abs_mm)

            rospy.loginfo_throttle(
                0.5,
                f"FAST 回零闭环: signed={current_signed_mm:+.1f} mm, "
                f"abs={current_abs_mm:.1f} mm, tol={tolerance_mm:.1f} mm, "
                f"cross_track={axis['cross_track_mm']:.1f} mm, "
                f"axis={axis['source']}"
                f"{self._format_return_x_error_text(axis)}"
            )

            if current_abs_mm <= tolerance_mm:
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name="倒车FAST停车")
                time.sleep(self.stop_settle_sec)
                final_signed_mm, _, final_axis = self.get_signed_fast_offset_mm()
                if final_signed_mm is None:
                    rospy.logwarn("FAST 回零停稳后: 无法读取最终有符号位移")
                    continue
                final_signed_mm = float(final_signed_mm)
                final_abs_mm = abs(final_signed_mm)
                rospy.loginfo(
                    f"FAST 回零停稳检查: final_signed={final_signed_mm:+.1f} mm, "
                    f"final_abs={final_abs_mm:.1f} mm, tol={tolerance_mm:.1f} mm, "
                    f"cross_track={final_axis['cross_track_mm']:.1f} mm"
                    f"{self._format_return_x_error_text(final_axis)}"
                )
                if final_abs_mm <= tolerance_mm:
                    rospy.loginfo("FAST 主闭环回零完成")
                    return True
                rospy.logwarn(
                    f"FAST 回零停稳后仍超出容差，继续补偿: final_signed={final_signed_mm:+.1f} mm"
                )
                continue

            if (
                self.return_fast_move_away_abort_mm > 0
                and current_abs_mm > best_abs_signed_mm + self.return_fast_move_away_abort_mm
            ):
                rospy.logwarn(
                    f"FAST 主闭环回零失败: 有符号位移绝对值反向增大过多: "
                    f"current_abs={current_abs_mm:.1f} mm, best_abs={best_abs_signed_mm:.1f} mm"
                )
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name="倒车FAST停车")
                time.sleep(self.stop_settle_sec)
                return False

            phase_index += 1
            slowdown_threshold_mm = max(
                float(self.return_fast_slowdown_distance_mm),
                tolerance_mm * 2.0,
                float(self.return_stop_buffer_mm),
            )
            stop_entry_mm = tolerance_mm + float(self.return_fast_stop_lead_mm)
            move_toward_zero_sign = -1 if current_signed_mm > 0 else 1
            if current_abs_mm > slowdown_threshold_mm:
                cmd_speed = move_toward_zero_sign * reverse_speed_abs
                phase_name = (
                    f"FAST回零主退{phase_index}"
                    if move_toward_zero_sign < 0
                    else f"FAST回零主进{phase_index}"
                )
            else:
                cmd_speed = move_toward_zero_sign * correction_speed_abs
                phase_name = (
                    f"FAST回零细退{phase_index}"
                    if move_toward_zero_sign < 0
                    else f"FAST回零细进{phase_index}"
                )

            phase_start_signed_mm = current_signed_mm
            active_speed_abs = abs(int(cmd_speed))
            rospy.loginfo(
                f"{phase_name}: start_signed={phase_start_signed_mm:+.1f} mm, "
                f"cmd_speed={cmd_speed} mm/s, axis={axis['source']}"
                f"{self._format_return_x_error_text(axis)}"
            )
            if not self.send_speed_mode_command(cmd_speed, self.angular_speed_cmd):
                return False

            try:
                while self.running and not rospy.is_shutdown():
                    loop_signed_mm, _, loop_axis = self.get_signed_fast_offset_mm()
                    if loop_signed_mm is None:
                        rospy.logwarn_throttle(
                            1.0,
                            f"{phase_name}: 暂时读不到 {self.heading_topic} 的有符号位移，保持当前速度继续等待..."
                        )
                        time.sleep(0.02)
                        continue

                    loop_signed_mm = float(loop_signed_mm)
                    loop_abs_mm = abs(loop_signed_mm)
                    best_abs_signed_mm = min(best_abs_signed_mm, loop_abs_mm)
                    rospy.loginfo_throttle(
                        0.5,
                        f"{phase_name}: signed={loop_signed_mm:+.1f} mm, "
                        f"abs={loop_abs_mm:.1f} mm, tol={tolerance_mm:.1f} mm, "
                        f"cross_track={loop_axis['cross_track_mm']:.1f} mm"
                        f"{self._format_return_x_error_text(loop_axis)}"
                    )

                    if loop_abs_mm <= tolerance_mm:
                        rospy.loginfo(f"{phase_name}: 已进入容差区间")
                        break

                    if active_speed_abs <= correction_speed_abs and loop_abs_mm <= stop_entry_mm:
                        rospy.loginfo(
                            f"{phase_name}: 已进入提前停车区间 abs={loop_abs_mm:.1f} mm "
                            f"<= stop_entry={stop_entry_mm:.1f} mm，停车后复查"
                        )
                        break

                    if phase_start_signed_mm > 0.0 and loop_signed_mm <= 0.0:
                        rospy.loginfo(f"{phase_name}: 检测到已越过起点，准备停车后再次修正")
                        break

                    if phase_start_signed_mm < 0.0 and loop_signed_mm >= 0.0:
                        rospy.loginfo(f"{phase_name}: 检测到已越过起点，准备停车后再次修正")
                        break

                    if active_speed_abs > correction_speed_abs and loop_abs_mm <= slowdown_threshold_mm:
                        slow_cmd_speed = move_toward_zero_sign * correction_speed_abs
                        slow_phase_name = (
                            f"FAST回零细退{phase_index}"
                            if move_toward_zero_sign < 0
                            else f"FAST回零细进{phase_index}"
                        )
                        rospy.loginfo(
                            f"{phase_name}: 进入减速区间 abs={loop_abs_mm:.1f} mm "
                            f"<= slowdown={slowdown_threshold_mm:.1f} mm, "
                            f"切换为 {slow_phase_name}, cmd_speed={slow_cmd_speed} mm/s"
                        )
                        if not self.send_speed_mode_command(slow_cmd_speed, self.angular_speed_cmd):
                            return False
                        phase_name = slow_phase_name
                        active_speed_abs = abs(int(slow_cmd_speed))
                        time.sleep(0.02)
                        continue

                    if (
                        self.return_fast_move_away_abort_mm > 0
                        and loop_abs_mm > best_abs_signed_mm + self.return_fast_move_away_abort_mm
                    ):
                        rospy.logwarn(
                            f"{phase_name}: 有符号位移绝对值反向增大过多: "
                            f"current_abs={loop_abs_mm:.1f} mm, best_abs={best_abs_signed_mm:.1f} mm"
                        )
                        break

                    time.sleep(0.02)
            finally:
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
                time.sleep(self.stop_settle_sec)

        self.stop_speed_motion()
        return False

    def run_reverse_phase(self):
        rospy.loginfo("=" * 60)
        if self.return_use_fast_position:
            rospy.loginfo("阶段2: FAST 主闭环回零")
            rospy.loginfo("=" * 60)
            fast_result = self.run_reverse_phase_fast_primary()
            if fast_result is True:
                return True

            if fast_result is None:
                rospy.logwarn("FAST 主回零不可用，当前已去除轮式里程计回退")
            else:
                rospy.logwarn("FAST 主回零执行失败，当前已去除轮式里程计回退")
            return False

        rospy.logerr("阶段2: FAST 主回零已关闭，但当前版本不再支持时间兜底回零")
        rospy.loginfo("=" * 60)
        return False

    def execute_sequence(self):
        success = False
        with self.trigger_lock:
            self.sequence_busy = True
            self.sequence_counter += 1
            sequence_id = self.sequence_counter
        self.current_cycle_pre_uprail_angle_deg = None

        try:
            rospy.loginfo("\n" + "=" * 70)
            rospy.loginfo(f"开始执行上轨往返流程，第 {sequence_id} 轮")
            rospy.loginfo("=" * 70)

            rospy.loginfo("阶段0: 建立/检查首次 SLAM 基准角")
            if not self.ensure_heading_reference():
                rospy.logerr("建立/检查首次 SLAM 基准角失败")
                return False

            if not self.switch_mode_if_needed(self.MODE_DRIVE):
                rospy.logerr("切换到行驶模式失败")
                return False

            if not self.run_forward_phase():
                rospy.logerr("前进阶段失败")
                return False

            if not self.run_reverse_phase():
                rospy.logerr("倒车阶段失败")
                return False

            self.log_heading_snapshot("下轨完成后")

            rospy.loginfo("阶段3: 下轨后先恢复到首次 SLAM 基准角")
            if not self.execute_post_return_angle_restore():
                rospy.logwarn("恢复首次 SLAM 基准角失败")
                return False

            if self.should_run_post_return_lateral_move(sequence_id):
                self.log_heading_snapshot("平移前姿态检查")
                rospy.loginfo("阶段4: 姿态修正后切到横移模式并执行横移位置控制")
                if not self.switch_mode_if_needed(self.MODE_LATERAL):
                    rospy.logwarn("切到横移模式失败")
                    return False
                if not self.execute_post_return_lateral_move(sequence_id):
                    rospy.logwarn("回零后横移位置控制失败")
                    return False
            else:
                rospy.loginfo("阶段4: 当前已是最后一轮，姿态修正后不再执行横移")

            rospy.loginfo("阶段5: 切回等待模式")
            if not self.switch_mode_if_needed(self.MODE_LATERAL):
                rospy.logwarn("切回等待模式失败")
                return False

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
            finally:
                self.can_bus = None


def signal_handler(sig, frame):
    rospy.signal_shutdown("用户终止")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    controller = UpRailFlagController()
    controller.run()


if __name__ == "__main__":
    main()
