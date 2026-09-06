#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 Flag_4_2.py 抽出来的“入轨 + 分段工作 + 回零”单独测试脚本。

默认测试流程:
1. 启动后切到等待模式，方便人工先把小车与铁轨对齐
2. 不再等待 /flag1，默认启动后自动执行一轮
3. 记录当前 FAST 起点
4. 先前进到 0.2m 作为第一个工作点
5. 后续每前进 0.2m 停一次，发送与原流程一致的采摘状态 CAN
   - 运动中: 0x20 -> busy
   - 到位后: 0x20 -> ready
6. 每个工作点等待 0x21 采摘完成，再继续前进到下一个工作点
7. 一直执行到总前进距离 10.0m
8. 使用与 Flag_4_2.py 相同的 FAST 主闭环回零逻辑退回起点
9. 切回等待模式，并继续发布 /rail_cycle_done 成功/失败信号

说明:
- 默认仅做“前进工作 + 回零”测试，不做姿态恢复、不做横移。
- 若夜间不想等采摘完成，可通过 ~skip_harvest_wait:=true 改回跳过等待。
- 若需要改距离，可通过参数覆盖:
  ~work_area_entry_distance_mm / ~work_step_mm / ~work_total_distance_mm
"""

import signal
import threading
import time

import rospy
from Flag_4_2 import can
from Flag_4_2 import UpRailFlagController, signal_handler


class UpRailReturnTestController(UpRailFlagController):
    def __init__(self):
        super().__init__()

        self.auto_start = bool(rospy.get_param("~auto_start", True))
        self.single_shot_exit = bool(rospy.get_param("~single_shot_exit", True))
        self.skip_harvest_wait = bool(rospy.get_param("~skip_harvest_wait", False))
        self.harvest_ready_hold_sec = max(
            0.0,
            float(rospy.get_param("~harvest_ready_hold_sec", 0.5)),
        )
        self.run_angle_restore = bool(rospy.get_param("~run_angle_restore", False))
        self.use_wait_mode = bool(rospy.get_param("~use_wait_mode", False))
        self.test_wait_mode_before_start = bool(
            rospy.get_param("~test_wait_mode_before_start", self.use_wait_mode)
        )
        self.startup_force_drive_mode = bool(
            rospy.get_param("~startup_force_drive_mode", not self.test_wait_mode_before_start)
        )
        self.force_drive_mode_before_forward = bool(
            rospy.get_param("~force_drive_mode_before_forward", True)
        )
        self.switch_to_wait_mode_on_finish = bool(
            rospy.get_param("~switch_to_wait_mode_on_finish", self.use_wait_mode)
        )
        self.work_area_entry_distance_mm = max(
            0.0,
            float(rospy.get_param("~work_area_entry_distance_mm", 200.0)),
        )
        self.work_step_mm = max(1.0, float(rospy.get_param("~work_step_mm", 200.0)))
        self.work_total_distance_mm = max(
            0.0,
            float(rospy.get_param("~work_total_distance_mm", 10000.0)),
        )
        self.work_stop_tolerance_mm = max(
            1.0,
            float(rospy.get_param("~work_stop_tolerance_mm", 100.0)),
        )
        self.first_work_stop_at_entry = bool(
            rospy.get_param("~first_work_stop_at_entry", True)
        )
        self.work_zone_linear_speed_mm_s = max(
            1,
            int(
                rospy.get_param(
                    "~work_zone_linear_speed_mm_s",
                    min(abs(int(self.forward_linear_speed_mm_s)), 300),
                )
            ),
        )
        self.work_correction_speed_mm_s = max(
            1,
            int(
                rospy.get_param(
                    "~work_correction_speed_mm_s",
                    min(self.work_zone_linear_speed_mm_s, max(60, abs(int(self.return_correction_speed_mm_s)))),
                )
            ),
        )
        self.work_point_slowdown_distance_mm = max(
            self.work_stop_tolerance_mm * 2.0,
            float(
                rospy.get_param(
                    "~work_point_slowdown_distance_mm",
                    max(self.work_stop_tolerance_mm * 3.0, 180.0),
                )
            ),
        )
        self.work_move_away_abort_mm = max(
            0.0,
            float(rospy.get_param("~work_move_away_abort_mm", 500.0)),
        )
        self.accept_short_harvest_done_payload = bool(
            rospy.get_param("~accept_short_harvest_done_payload", True)
        )
        self.harvest_status_repeat_sec = max(
            0.0,
            float(
                rospy.get_param(
                    "~harvest_status_repeat_sec",
                    rospy.get_param("~harvest_busy_repeat_sec", 0.2),
                )
            ),
        )
        self.forward_distance_mm = self.work_total_distance_mm
        self._harvest_status_repeat_lock = threading.RLock()
        self._harvest_status_repeat_mode = None
        self._last_status_repeat_wall_time = 0.0
        self._status_repeat_thread = None

        rospy.loginfo("=" * 70)
        rospy.loginfo("入轨+分段工作+回零单测脚本已切换到独立模式")
        rospy.loginfo(f"  自动启动: {'启用' if self.auto_start else '禁用'}")
        rospy.loginfo(f"  单轮后退出: {'启用' if self.single_shot_exit else '禁用'}")
        rospy.loginfo(
            f"  工作区入口距离: {self.work_area_entry_distance_mm:.1f} mm, "
            f"首点计入工作点: {'是' if self.first_work_stop_at_entry else '否'}"
        )
        rospy.loginfo(f"  工作步进距离: {self.work_step_mm:.1f} mm")
        rospy.loginfo(f"  总前进距离: {self.work_total_distance_mm:.1f} mm")
        rospy.loginfo(f"  工作点容差: {self.work_stop_tolerance_mm:.1f} mm")
        rospy.loginfo(f"  入轨速度: {abs(int(self.forward_linear_speed_mm_s))} mm/s")
        rospy.loginfo(
            f"  工作区速度(入口点 {self.work_area_entry_distance_mm:.1f} mm 后): "
            f"{self.work_zone_linear_speed_mm_s} mm/s"
        )
        rospy.loginfo(f"  工作点补偿速度: {self.work_correction_speed_mm_s} mm/s")
        rospy.loginfo(
            f"  采摘完成等待: {'跳过' if self.skip_harvest_wait else '保留原流程等待'}"
        )
        rospy.loginfo(f"  采摘状态重复发送周期: {self.harvest_status_repeat_sec:.2f}s")
        if self.skip_harvest_wait:
            rospy.loginfo(f"  到位 ready 保持时间: {self.harvest_ready_hold_sec:.2f}s")
        else:
            rospy.loginfo(
                f"  采摘完成短帧兼容: {'启用' if self.accept_short_harvest_done_payload else '禁用'}"
            )
        rospy.loginfo(f"  姿态恢复: {'启用' if self.run_angle_restore else '禁用'}")
        rospy.loginfo(
            f"  等待模式切换: {'启用' if self.use_wait_mode else '禁用，直接按行驶模式测试'}"
        )
        rospy.loginfo(
            f"  启动先切自动/行驶模式: {'启用' if self.startup_force_drive_mode else '禁用'}"
        )
        rospy.loginfo(
            f"  前进前补发自动/行驶模式: {'启用' if self.force_drive_mode_before_forward else '禁用'}"
        )
        rospy.loginfo("  横移: 已禁用")
        rospy.loginfo("=" * 70)

        if self.harvest_status_repeat_sec > 0.0:
            self._status_repeat_thread = threading.Thread(
                target=self._status_repeat_loop,
                name="harvest_status_repeat",
                daemon=True,
            )
            self._status_repeat_thread.start()

    def _build_work_segment_distances_mm(self):
        total_mm = float(self.work_total_distance_mm)
        if total_mm <= 0.0:
            return []

        entry_mm = min(float(self.work_area_entry_distance_mm), total_mm)
        step_mm = max(1.0, float(self.work_step_mm))
        segments = []
        covered_mm = 0.0

        if entry_mm > 0.0 and self.first_work_stop_at_entry:
            segments.append(entry_mm)
            covered_mm = entry_mm

        while covered_mm < total_mm - 1e-6:
            next_segment_mm = min(step_mm, total_mm - covered_mm)
            if next_segment_mm <= 1e-6:
                break
            segments.append(next_segment_mm)
            covered_mm += next_segment_mm

        return [float(round(segment_mm, 3)) for segment_mm in segments]

    def _set_harvest_status_repeat_mode(self, mode):
        with self._harvest_status_repeat_lock:
            self._harvest_status_repeat_mode = mode
            if mode is None:
                self._last_status_repeat_wall_time = 0.0

    def send_harvest_status(self, ready):
        with self._harvest_status_repeat_lock:
            sent = super().send_harvest_status(ready)
            if sent:
                self._last_status_repeat_wall_time = time.time()
            return sent

    def _send_status_repeat(self, mode):
        with self._harvest_status_repeat_lock:
            if mode != self._harvest_status_repeat_mode:
                return False
            if mode == "ready":
                payload = self.harvest_ready_payload
            elif mode == "busy":
                payload = self.harvest_busy_payload
            else:
                return False

            sent = self.send_can_message(payload, arbitration_id=self.harvest_status_can_id)
            if sent:
                if mode == "ready":
                    self.harvest_ready_sent_wall_time = time.time()
                else:
                    self.harvest_ready_sent_wall_time = 0.0
            return sent

    def _status_repeat_loop(self):
        while self.running and not rospy.is_shutdown():
            with self._harvest_status_repeat_lock:
                repeat_mode = self._harvest_status_repeat_mode
                last_send = self._last_status_repeat_wall_time
            if repeat_mode is None or self.harvest_status_repeat_sec <= 0.0:
                time.sleep(0.02)
                continue

            now = time.time()
            if last_send > 0.0 and now - last_send < self.harvest_status_repeat_sec:
                time.sleep(0.02)
                continue

            sent = self._send_status_repeat(repeat_mode)
            if sent:
                with self._harvest_status_repeat_lock:
                    self._last_status_repeat_wall_time = time.time()
            time.sleep(0.02)

    def _force_drive_mode_before_forward(self):
        if self.can_bus is None or can is None:
            self.current_mode = self.MODE_DRIVE
            rospy.loginfo("[模拟CAN] 前进前默认切到自动/行驶模式")
            return True

        rospy.loginfo("前进前先显式发送自动/行驶模式切换命令")
        if not self.send_can_message([0x05, 0, 0, self.MODE_DRIVE, 0, 0, 0, 0]):
            rospy.logerr("前进前发送自动/行驶模式命令失败")
            return False

        time.sleep(max(0.05, self.MODE_SWITCH_QUERY_INTERVAL))
        if not self.switch_mode_if_needed(self.MODE_DRIVE):
            rospy.logerr("前进前确认自动/行驶模式失败")
            return False

        return True

    def send_speed_mode_command(self, linear_mm_s, angular_cmd=0):
        if int(linear_mm_s) > 0 and self.force_drive_mode_before_forward:
            if not self._force_drive_mode_before_forward():
                rospy.logerr("取消本次前进速度命令: 自动/行驶模式未就绪")
                return False

        success = super().send_speed_mode_command(linear_mm_s, angular_cmd)
        if not success:
            return False

        moving = int(linear_mm_s) != 0 or int(angular_cmd) != 0
        if moving:
            self.send_harvest_status(False)
            self._set_harvest_status_repeat_mode("busy")
        else:
            self._set_harvest_status_repeat_mode(None)
        return True

    def stop_speed_motion(self):
        self._set_harvest_status_repeat_mode(None)
        return super().stop_speed_motion()

    def wait_for_harvest_complete(self, phase_name):
        self._set_harvest_status_repeat_mode("ready")
        if not self.skip_harvest_wait:
            if self.can_bus is None or can is None:
                self._set_harvest_status_repeat_mode(None)
                rospy.loginfo(f"{phase_name}: [模拟CAN] 默认采摘完成，继续执行")
                return True

            timeout_sec = float(self.harvest_wait_timeout_sec)
            deadline = None if timeout_sec <= 0.0 else time.time() + timeout_sec
            accept_after = max(
                float(self.harvest_ready_sent_wall_time) + float(self.harvest_arm_delay_sec),
                float(self.harvest_done_ignore_until),
            )
            expected_short_payload = bytes([self.harvest_done_payload[0]])
            rospy.loginfo(
                f"{phase_name}: 等待采摘完成信号 can_id=0x{self.harvest_done_can_id:03X}, "
                f"data={self._format_can_data(self.harvest_done_payload)}, "
                f"timeout={'disabled' if deadline is None else f'{timeout_sec:.1f}s'}, "
                f"accept_after={accept_after:.3f}, "
                f"short_payload={'enabled' if self.accept_short_harvest_done_payload else 'disabled'}"
            )

            while self.running and not rospy.is_shutdown():
                entry = self._pop_harvest_frame()
                if entry is None:
                    if deadline is not None and time.time() >= deadline:
                        self._set_harvest_status_repeat_mode(None)
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
                short_payload_ok = (
                    self.accept_short_harvest_done_payload
                    and payload == expected_short_payload
                )
                if payload == self.harvest_done_payload or short_payload_ok:
                    self.harvest_done_ignore_until = time.time() + float(self.harvest_duplicate_suppress_sec)
                    self._clear_harvest_rx_queue()
                    self._set_harvest_status_repeat_mode(None)
                    self.send_harvest_status(False)
                    suffix = " (短帧兼容)" if short_payload_ok else ""
                    rospy.loginfo(
                        f"{phase_name}: 收到采摘完成信号 can_id=0x{int(msg.arbitration_id):03X}, "
                        f"data={self._format_can_data(payload)}{suffix}"
                    )
                    return True

                rospy.logwarn(
                    f"{phase_name}: 忽略非预期采摘信号 can_id=0x{int(msg.arbitration_id):03X}, "
                    f"data={self._format_can_data(payload)}"
                )
            self._set_harvest_status_repeat_mode(None)
            return False

        if self.harvest_ready_hold_sec > 0.0:
            rospy.loginfo(
                f"{phase_name}: 测试模式跳过 0x21 采摘完成等待，"
                f"仅保留 ready 状态 {self.harvest_ready_hold_sec:.2f}s"
            )
            time.sleep(self.harvest_ready_hold_sec)
        else:
            rospy.loginfo(f"{phase_name}: 测试模式跳过 0x21 采摘完成等待")
        self._set_harvest_status_repeat_mode(None)
        return True

    def wait_for_harvest_rearm_window(self, phase_name):
        if self.skip_harvest_wait:
            return True
        return super().wait_for_harvest_rearm_window(phase_name)

    def _get_fast_distance_from_start_mm(self):
        fast_distance_mm, _ = self.get_distance_to_fast_start_mm()
        if fast_distance_mm is None:
            return None
        return max(0.0, abs(float(fast_distance_mm)))

    def _choose_work_cmd_speed(self, error_mm, absolute_target_mm):
        base_speed_abs = (
            abs(int(self.forward_linear_speed_mm_s))
            if absolute_target_mm <= self.work_area_entry_distance_mm + 1e-6
            else int(self.work_zone_linear_speed_mm_s)
        )
        correction_speed_abs = min(base_speed_abs, int(self.work_correction_speed_mm_s))
        abs_error_mm = abs(float(error_mm))
        speed_abs = base_speed_abs if abs_error_mm > self.work_point_slowdown_distance_mm else correction_speed_abs
        return speed_abs if error_mm > 0.0 else -speed_abs

    def _move_to_target_distance_mm(self, absolute_target_mm, phase_name):
        tolerance_mm = float(self.work_stop_tolerance_mm)
        best_abs_error_mm = None
        last_cmd_speed = None

        while self.running and not rospy.is_shutdown():
            current_distance_mm = self._get_fast_distance_from_start_mm()
            if current_distance_mm is None:
                rospy.logwarn_throttle(
                    1.0,
                    f"{phase_name}: 暂时无法读取 FAST 距离，继续等待 {self.heading_topic}"
                )
                time.sleep(0.02)
                continue

            error_mm = float(absolute_target_mm) - float(current_distance_mm)
            abs_error_mm = abs(error_mm)
            if best_abs_error_mm is None or abs_error_mm < best_abs_error_mm:
                best_abs_error_mm = abs_error_mm

            rospy.loginfo_throttle(
                0.4,
                f"{phase_name}: slam_dist_mm={current_distance_mm:.1f}, "
                f"target_mm={absolute_target_mm:.1f}, error_mm={error_mm:+.1f}, "
                f"tol={tolerance_mm:.1f}"
            )

            if abs_error_mm <= tolerance_mm:
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
                time.sleep(self.stop_settle_sec)
                verified_distance_mm = self._get_fast_distance_from_start_mm()
                if verified_distance_mm is None:
                    rospy.logwarn(f"{phase_name}: 停稳后无法复核 FAST 距离，继续等待复测")
                    continue
                verified_error_mm = float(absolute_target_mm) - float(verified_distance_mm)
                if abs(verified_error_mm) <= tolerance_mm:
                    rospy.loginfo(
                        f"{phase_name}: 已修正到容差内, slam_dist_mm={verified_distance_mm:.1f}, "
                        f"target_mm={absolute_target_mm:.1f}, error_mm={verified_error_mm:+.1f}"
                    )
                    return float(verified_distance_mm)

                rospy.logwarn(
                    f"{phase_name}: 停稳复核仍超出容差, slam_dist_mm={verified_distance_mm:.1f}, "
                    f"target_mm={absolute_target_mm:.1f}, error_mm={verified_error_mm:+.1f}, 继续补偿"
                )
                best_abs_error_mm = min(best_abs_error_mm, abs(verified_error_mm))
                last_cmd_speed = None
                continue

            if (
                self.work_move_away_abort_mm > 0.0
                and best_abs_error_mm is not None
                and abs_error_mm > best_abs_error_mm + self.work_move_away_abort_mm
            ):
                rospy.logerr(
                    f"{phase_name}: 与目标误差反向增大过多, current_error={error_mm:+.1f} mm, "
                    f"best_abs_error={best_abs_error_mm:.1f} mm"
                )
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
                time.sleep(self.stop_settle_sec)
                return None

            cmd_speed = self._choose_work_cmd_speed(error_mm, absolute_target_mm)
            if last_cmd_speed != cmd_speed:
                phase_text = "前进" if cmd_speed > 0 else "回修"
                rospy.loginfo(
                    f"{phase_name}: 切换补偿命令 -> {phase_text}, cmd_speed={cmd_speed} mm/s, "
                    f"slam_dist_mm={current_distance_mm:.1f}, target_mm={absolute_target_mm:.1f}, "
                    f"error_mm={error_mm:+.1f}"
                )
                if not self.send_speed_mode_command(cmd_speed, self.angular_speed_cmd):
                    return None
                last_cmd_speed = cmd_speed

            time.sleep(0.02)

        self.stop_speed_motion()
        return None

    def run_forward_phase(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("阶段1: 入轨后按工作点序列执行前进 + 等待采摘")
        rospy.loginfo("=" * 60)

        self.record_start_odometry()
        if self.start_fast_position is None:
            rospy.logerr("前进阶段失败: 未记录到有效起点 FAST 位姿")
            return False

        segment_distances_mm = self._build_work_segment_distances_mm()
        if not segment_distances_mm:
            rospy.loginfo("前进目标距离为 0，跳过前进")
            return True

        rospy.loginfo(
            f"本轮分段概要: 首段={segment_distances_mm[0]:.1f} mm, "
            f"标准步进={self.work_step_mm:.1f} mm, "
            f"分段数={len(segment_distances_mm)}, "
            f"总前进={sum(segment_distances_mm):.1f} mm"
        )

        if not self.send_harvest_status(False):
            return False

        confirmed_distance_mm = 0.0
        for index, segment_distance_mm in enumerate(segment_distances_mm, start=1):
            phase_name = f"工作点{index}"
            current_distance_mm = self._get_fast_distance_from_start_mm()
            if current_distance_mm is None:
                current_distance_mm = confirmed_distance_mm
            absolute_target_mm = confirmed_distance_mm + segment_distance_mm
            rospy.loginfo(
                f"{phase_name}: 当前累计={current_distance_mm:.1f} mm, "
                f"上一确认点={confirmed_distance_mm:.1f} mm, "
                f"本段目标={segment_distance_mm:.1f} mm, "
                f"本段目标累计={absolute_target_mm:.1f} mm"
            )

            current_distance_mm = self._move_to_target_distance_mm(absolute_target_mm, phase_name)
            if current_distance_mm is None:
                return False

            actual_segment_mm = current_distance_mm - confirmed_distance_mm
            segment_error_mm = actual_segment_mm - segment_distance_mm
            if abs(segment_error_mm) <= self.work_stop_tolerance_mm:
                rospy.loginfo(
                    f"{phase_name}: 实际本段位移={actual_segment_mm:.1f} mm, "
                    f"目标本段位移={segment_distance_mm:.1f} mm, "
                    f"本段误差={segment_error_mm:+.1f} mm, 已在容差内"
                )
            else:
                rospy.logwarn(
                    f"{phase_name}: 实际本段位移={actual_segment_mm:.1f} mm, "
                    f"目标本段位移={segment_distance_mm:.1f} mm, "
                    f"本段误差={segment_error_mm:+.1f} mm, "
                    f"超出容差={self.work_stop_tolerance_mm:.1f} mm"
                )
            confirmed_distance_mm = current_distance_mm

            if not self.wait_for_harvest_rearm_window(phase_name):
                return False
            if not self.send_harvest_status(True):
                return False
            if not self.wait_for_harvest_complete(phase_name):
                return False

            if index < len(segment_distances_mm):
                rospy.loginfo(
                    f"{phase_name}: 采摘完成，继续前进下一段 {segment_distances_mm[index]:.1f} mm"
                )
                if not self.send_harvest_status(False):
                    return False

        if not self.send_harvest_status(False):
            return False

        self.record_forward_end_odometry()
        return True

    def execute_sequence(self):
        success = False
        with self.trigger_lock:
            self.sequence_busy = True
            self.sequence_counter += 1
            sequence_id = self.sequence_counter
        self.current_cycle_pre_uprail_angle_deg = None

        try:
            rospy.loginfo("\n" + "=" * 70)
            rospy.loginfo(f"开始执行入轨+分段工作+回零单测，第 {sequence_id} 轮")
            rospy.loginfo("=" * 70)

            if self.test_wait_mode_before_start:
                rospy.loginfo("阶段0: 保持等待模式，默认由人工完成轨道对齐")
                if not self.switch_mode_if_needed(self.MODE_LATERAL):
                    rospy.logerr("切到等待模式失败")
                    return False
            else:
                rospy.loginfo("阶段0: 跳过等待模式，直接进入行驶模式测试")

            rospy.loginfo("阶段1: 切到行驶模式，前进到测试距离")
            if not self.switch_mode_if_needed(self.MODE_DRIVE):
                rospy.logerr("切换到行驶模式失败")
                return False

            if not self.run_forward_phase():
                rospy.logerr("前进阶段失败")
                return False

            rospy.loginfo("阶段2: 使用 FAST 主闭环回零")
            if not self.run_reverse_phase():
                rospy.logerr("回零阶段失败")
                return False

            if self.run_angle_restore:
                rospy.loginfo("阶段3: 执行姿态恢复")
                if not self.ensure_heading_reference():
                    rospy.logerr("姿态恢复前建立基准角失败")
                    return False
                if not self.execute_post_return_angle_restore():
                    rospy.logerr("姿态恢复失败")
                    return False

            if self.switch_to_wait_mode_on_finish:
                rospy.loginfo("阶段4: 切回等待模式")
                if not self.switch_mode_if_needed(self.MODE_LATERAL):
                    rospy.logwarn("切回等待模式失败")
                    return False
            else:
                rospy.loginfo("阶段4: 跳过等待模式切换，保持当前模式停车结束")

            rospy.loginfo("入轨+分段工作+回零单测完成")
            success = True
            return True

        finally:
            self.stop_speed_motion()
            with self.trigger_lock:
                self.sequence_busy = False
                self.trigger_requested = False
            self.publish_cycle_done(sequence_id, success)

    def run(self):
        if self.test_wait_mode_before_start:
            rospy.loginfo("启动后先切到等待模式，便于人工对齐后再测")
            self.switch_mode_if_needed(self.MODE_LATERAL)
        else:
            rospy.loginfo("启动后跳过等待模式，直接准备进入自动/行驶模式")
            if self.startup_force_drive_mode:
                if not self._force_drive_mode_before_forward():
                    rospy.logwarn("启动阶段切自动/行驶模式失败，前进前会继续重试")

        if self.auto_start:
            rospy.loginfo("测试模式: 启动后自动执行单轮入轨+工作+回零")
            self.execute_sequence()
            if self.single_shot_exit:
                rospy.loginfo("测试模式: 单轮执行结束，脚本退出")
                return

        super().run()

    def shutdown(self):
        self._set_harvest_status_repeat_mode(None)
        super().shutdown()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    controller = UpRailReturnTestController()
    controller.run()


if __name__ == "__main__":
    main()
