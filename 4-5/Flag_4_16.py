#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于 Flag_4_2.py 的 4-16 版本。

仅替换前进阶段逻辑，其余流程保持沿用 Flag_4_2.py：
- 第一次前进距离可单独配置
- 后续每次前进距离可单独配置
- 每段到位后继续使用原有采摘 CAN 交互
- 所有前进段完成后，继续走原来的 FAST 主闭环回零、姿态恢复、横移与完成信号
- 每次发送正向前进速度前，先补一次自动/行驶模式切换，沿用测试版里验证过的保护
"""

import signal
import time

import rospy

from Flag_4_2 import UpRailFlagController, can, signal_handler


class UpRail416Controller(UpRailFlagController):
    def __init__(self):
        super().__init__()

        self.forward_target_x_offsets_m = self._load_target_x_offsets_m()
        self.forward_first_segment_mm = max(
            0.0,
            float(
                rospy.get_param(
                    "~forward_first_segment_mm",
                    rospy.get_param(
                        "~work_area_entry_distance_mm",
                        min(float(self.forward_pick_step_mm), float(self.forward_distance_mm)),
                    ),
                )
            ),
        )
        self.forward_segment_mm = max(
            1.0,
            float(
                rospy.get_param(
                    "~forward_segment_mm",
                    rospy.get_param("~work_step_mm", float(self.forward_pick_step_mm)),
                )
            ),
        )
        self.first_forward_stop_at_entry = bool(
            rospy.get_param(
                "~first_forward_stop_at_entry",
                rospy.get_param("~first_work_stop_at_entry", True),
            )
        )
        self.forward_stop_tolerance_mm = max(
            1.0,
            float(
                rospy.get_param(
                    "~forward_stop_tolerance_mm",
                    rospy.get_param("~work_stop_tolerance_mm", 50.0),
                )
            ),
        )
        self.forward_zone_linear_speed_mm_s = max(
            1,
            int(
                rospy.get_param(
                    "~forward_zone_linear_speed_mm_s",
                    rospy.get_param(
                        "~work_zone_linear_speed_mm_s",
                        abs(int(self.forward_linear_speed_mm_s)),
                    ),
                )
            ),
        )
        self.forward_correction_speed_mm_s = max(
            1,
            int(
                rospy.get_param(
                    "~forward_correction_speed_mm_s",
                    rospy.get_param(
                        "~work_correction_speed_mm_s",
                        min(
                            int(self.forward_zone_linear_speed_mm_s),
                            max(60, abs(int(self.return_correction_speed_mm_s))),
                        ),
                    ),
                )
            ),
        )
        self.forward_point_slowdown_distance_mm = max(
            self.forward_stop_tolerance_mm * 2.0,
            float(
                rospy.get_param(
                    "~forward_point_slowdown_distance_mm",
                    rospy.get_param(
                        "~work_point_slowdown_distance_mm",
                        max(self.forward_stop_tolerance_mm * 3.0, 180.0),
                    ),
                )
            ),
        )
        self.forward_move_away_abort_mm = max(
            0.0,
            float(
                rospy.get_param(
                    "~forward_move_away_abort_mm",
                    rospy.get_param("~work_move_away_abort_mm", 500.0),
                )
            ),
        )
        self.force_drive_mode_before_forward = bool(
            rospy.get_param("~force_drive_mode_before_forward", True)
        )

        rospy.loginfo("=" * 70)
        rospy.loginfo("4-16 前进阶段增强已启用")
        if self.forward_target_x_offsets_m:
            rospy.loginfo(
                "  累计目标X序列(m): "
                + ", ".join(f"{value:.3f}" for value in self.forward_target_x_offsets_m)
            )
            rospy.loginfo("  前进控制模式: 基于起始点累计 X 变化量闭环")
        else:
            rospy.loginfo("  前进控制模式: 基于 FAST 平面距离分段闭环")
        rospy.loginfo(f"  首次前进距离: {self.forward_first_segment_mm:.1f} mm")
        rospy.loginfo(f"  后续步进距离: {self.forward_segment_mm:.1f} mm")
        rospy.loginfo(f"  总前进距离: {self.forward_distance_mm:.1f} mm")
        rospy.loginfo(f"  首点单独停车: {'启用' if self.first_forward_stop_at_entry else '禁用'}")
        rospy.loginfo(f"  前进停车容差: {self.forward_stop_tolerance_mm:.1f} mm")
        rospy.loginfo(f"  前进基准速度: {abs(int(self.forward_linear_speed_mm_s))} mm/s")
        rospy.loginfo(f"  工作段速度: {self.forward_zone_linear_speed_mm_s} mm/s")
        rospy.loginfo(f"  前进补偿速度: {self.forward_correction_speed_mm_s} mm/s")
        rospy.loginfo(
            f"  前进前补发自动/行驶模式: {'启用' if self.force_drive_mode_before_forward else '禁用'}"
        )
        rospy.loginfo("=" * 70)

    def _load_target_x_offsets_m(self):
        raw_value = rospy.get_param("~forward_target_x_offsets_m", None)
        if raw_value is None:
            raw_value = rospy.get_param("~forward_target_x_list", None)
        if raw_value is None:
            return []

        if isinstance(raw_value, str):
            tokens = [token.strip() for token in raw_value.replace(";", ",").split(",")]
            values = [token for token in tokens if token]
        elif isinstance(raw_value, (list, tuple)):
            values = list(raw_value)
        else:
            rospy.logwarn(f"forward_target_x_offsets_m 参数格式不支持: {type(raw_value)}，已忽略")
            return []

        target_list = []
        for index, value in enumerate(values, start=1):
            try:
                target_value = float(value)
            except (TypeError, ValueError):
                rospy.logwarn(f"累计目标X序列第{index}项无效: {value}，已忽略")
                continue
            target_list.append(target_value)

        target_list = sorted(target_list)
        return target_list

    def _get_fast_distance_from_start_mm(self):
        fast_distance_mm, _ = self.get_distance_to_fast_start_mm()
        if fast_distance_mm is None:
            return None
        return max(0.0, abs(float(fast_distance_mm)))

    def _get_fast_x_offset_from_start_mm(self):
        if self.start_fast_position is None:
            return None

        heading_snapshot = self.get_heading_yaw_deg()
        if heading_snapshot is None or heading_snapshot.get("position") is None:
            return None

        current_position = heading_snapshot["position"]
        return (float(current_position[0]) - float(self.start_fast_position[0])) * 1000.0

    def _build_forward_segment_distances_mm(self):
        total_mm = max(0.0, float(self.forward_distance_mm))
        if total_mm <= 0.0:
            return []

        first_segment_mm = min(float(self.forward_first_segment_mm), total_mm)
        step_mm = max(1.0, float(self.forward_segment_mm))
        segments = []
        covered_mm = 0.0

        if first_segment_mm > 0.0 and self.first_forward_stop_at_entry:
            segments.append(first_segment_mm)
            covered_mm = first_segment_mm

        while covered_mm < total_mm - 1e-6:
            next_segment_mm = min(step_mm, total_mm - covered_mm)
            if next_segment_mm <= 1e-6:
                break
            segments.append(next_segment_mm)
            covered_mm += next_segment_mm

        return [float(round(segment_mm, 3)) for segment_mm in segments]

    def _build_forward_target_x_offsets_mm(self):
        return [float(round(value * 1000.0, 3)) for value in self.forward_target_x_offsets_m]

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

    def _choose_forward_cmd_speed(self, error_mm, absolute_target_mm):
        base_speed_abs = (
            abs(int(self.forward_linear_speed_mm_s))
            if absolute_target_mm <= self.forward_first_segment_mm + 1e-6
            else int(self.forward_zone_linear_speed_mm_s)
        )
        correction_speed_abs = min(base_speed_abs, int(self.forward_correction_speed_mm_s))
        abs_error_mm = abs(float(error_mm))
        speed_abs = (
            base_speed_abs
            if abs_error_mm > self.forward_point_slowdown_distance_mm
            else correction_speed_abs
        )
        return speed_abs if error_mm > 0.0 else -speed_abs

    def _move_to_target_distance_mm(self, absolute_target_mm, phase_name):
        tolerance_mm = float(self.forward_stop_tolerance_mm)
        best_abs_error_mm = None
        last_cmd_speed = None

        while self.running and not rospy.is_shutdown():
            current_distance_mm = self._get_fast_distance_from_start_mm()
            if current_distance_mm is None:
                rospy.logwarn_throttle(
                    1.0,
                    f"{phase_name}: 暂时无法读取 FAST 距离，继续等待 {self.heading_topic}",
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
                f"tol={tolerance_mm:.1f}",
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
                        f"target_mm={absolute_target_mm:.1f}, error_mm={verified_error_mm:+.1f}",
                    )
                    return float(verified_distance_mm)

                rospy.logwarn(
                    f"{phase_name}: 停稳复核仍超出容差, slam_dist_mm={verified_distance_mm:.1f}, "
                    f"target_mm={absolute_target_mm:.1f}, error_mm={verified_error_mm:+.1f}, 继续补偿",
                )
                best_abs_error_mm = min(best_abs_error_mm, abs(verified_error_mm))
                last_cmd_speed = None
                continue

            if (
                self.forward_move_away_abort_mm > 0.0
                and best_abs_error_mm is not None
                and abs_error_mm > best_abs_error_mm + self.forward_move_away_abort_mm
            ):
                rospy.logerr(
                    f"{phase_name}: 与目标误差反向增大过多, current_error={error_mm:+.1f} mm, "
                    f"best_abs_error={best_abs_error_mm:.1f} mm",
                )
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
                time.sleep(self.stop_settle_sec)
                return None

            cmd_speed = self._choose_forward_cmd_speed(error_mm, absolute_target_mm)
            if last_cmd_speed != cmd_speed:
                phase_text = "前进" if cmd_speed > 0 else "回修"
                rospy.loginfo(
                    f"{phase_name}: 切换补偿命令 -> {phase_text}, cmd_speed={cmd_speed} mm/s, "
                    f"slam_dist_mm={current_distance_mm:.1f}, target_mm={absolute_target_mm:.1f}, "
                    f"error_mm={error_mm:+.1f}",
                )
                if cmd_speed > 0 and self.force_drive_mode_before_forward:
                    if not self._force_drive_mode_before_forward():
                        return None
                if not self.send_speed_mode_command(cmd_speed, self.angular_speed_cmd):
                    return None
                last_cmd_speed = cmd_speed

            time.sleep(0.02)

        self.stop_speed_motion()
        return None

    def _move_to_target_x_offset_mm(self, target_x_offset_mm, phase_name):
        tolerance_mm = float(self.forward_stop_tolerance_mm)
        best_abs_error_mm = None
        last_cmd_speed = None

        while self.running and not rospy.is_shutdown():
            current_x_offset_mm = self._get_fast_x_offset_from_start_mm()
            if current_x_offset_mm is None:
                rospy.logwarn_throttle(
                    1.0,
                    f"{phase_name}: 暂时无法读取 FAST X 偏移，继续等待 {self.heading_topic}",
                )
                time.sleep(0.02)
                continue

            error_mm = float(target_x_offset_mm) - float(current_x_offset_mm)
            abs_error_mm = abs(error_mm)
            if best_abs_error_mm is None or abs_error_mm < best_abs_error_mm:
                best_abs_error_mm = abs_error_mm

            rospy.loginfo_throttle(
                0.4,
                f"{phase_name}: slam_x_offset_mm={current_x_offset_mm:.1f}, "
                f"target_x_offset_mm={target_x_offset_mm:.1f}, error_mm={error_mm:+.1f}, "
                f"tol={tolerance_mm:.1f}",
            )

            if abs_error_mm <= tolerance_mm:
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
                time.sleep(self.stop_settle_sec)

                verified_x_offset_mm = self._get_fast_x_offset_from_start_mm()
                if verified_x_offset_mm is None:
                    rospy.logwarn(f"{phase_name}: 停稳后无法复核 FAST X 偏移，继续等待复测")
                    continue

                verified_error_mm = float(target_x_offset_mm) - float(verified_x_offset_mm)
                if abs(verified_error_mm) <= tolerance_mm:
                    rospy.loginfo(
                        f"{phase_name}: 已修正到容差内, slam_x_offset_mm={verified_x_offset_mm:.1f}, "
                        f"target_x_offset_mm={target_x_offset_mm:.1f}, error_mm={verified_error_mm:+.1f}",
                    )
                    return float(verified_x_offset_mm)

                rospy.logwarn(
                    f"{phase_name}: 停稳复核仍超出容差, slam_x_offset_mm={verified_x_offset_mm:.1f}, "
                    f"target_x_offset_mm={target_x_offset_mm:.1f}, error_mm={verified_error_mm:+.1f}, 继续补偿",
                )
                best_abs_error_mm = min(best_abs_error_mm, abs(verified_error_mm))
                last_cmd_speed = None
                continue

            if (
                self.forward_move_away_abort_mm > 0.0
                and best_abs_error_mm is not None
                and abs_error_mm > best_abs_error_mm + self.forward_move_away_abort_mm
            ):
                rospy.logerr(
                    f"{phase_name}: 与目标X误差反向增大过多, current_error={error_mm:+.1f} mm, "
                    f"best_abs_error={best_abs_error_mm:.1f} mm",
                )
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
                time.sleep(self.stop_settle_sec)
                return None

            cmd_speed = self._choose_forward_cmd_speed(error_mm, target_x_offset_mm)
            if last_cmd_speed != cmd_speed:
                phase_text = "前进" if cmd_speed > 0 else "回修"
                rospy.loginfo(
                    f"{phase_name}: 切换补偿命令 -> {phase_text}, cmd_speed={cmd_speed} mm/s, "
                    f"slam_x_offset_mm={current_x_offset_mm:.1f}, target_x_offset_mm={target_x_offset_mm:.1f}, "
                    f"error_mm={error_mm:+.1f}",
                )
                if cmd_speed > 0 and self.force_drive_mode_before_forward:
                    if not self._force_drive_mode_before_forward():
                        return None
                if not self.send_speed_mode_command(cmd_speed, self.angular_speed_cmd):
                    return None
                last_cmd_speed = cmd_speed

            time.sleep(0.02)

        self.stop_speed_motion()
        return None

    def run_forward_phase(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("阶段1: 使用 4-16 分段前进逻辑执行上轨")
        rospy.loginfo("=" * 60)

        self.record_start_odometry()
        if self.start_fast_position is None:
            rospy.logerr("前进阶段失败: 未记录到有效起点 FAST 位姿")
            return False

        if self.forward_target_x_offsets_m:
            return self._run_forward_phase_with_x_targets()

        segment_distances_mm = self._build_forward_segment_distances_mm()
        if not segment_distances_mm:
            rospy.loginfo("前进目标距离为 0，跳过前进")
            return True

        rospy.loginfo(
            f"本轮分段概要: 首段={segment_distances_mm[0]:.1f} mm, "
            f"后续步进={self.forward_segment_mm:.1f} mm, "
            f"分段数={len(segment_distances_mm)}, "
            f"总前进={sum(segment_distances_mm):.1f} mm",
        )

        if not self.send_harvest_status(False):
            return False

        confirmed_distance_mm = 0.0
        for index, segment_distance_mm in enumerate(segment_distances_mm, start=1):
            phase_name = f"前进分段{index}"
            current_distance_mm = self._get_fast_distance_from_start_mm()
            if current_distance_mm is None:
                current_distance_mm = confirmed_distance_mm
            absolute_target_mm = confirmed_distance_mm + segment_distance_mm

            rospy.loginfo(
                f"{phase_name}: 当前累计={current_distance_mm:.1f} mm, "
                f"上一确认点={confirmed_distance_mm:.1f} mm, "
                f"本段目标={segment_distance_mm:.1f} mm, "
                f"本段目标累计={absolute_target_mm:.1f} mm",
            )

            current_distance_mm = self._move_to_target_distance_mm(absolute_target_mm, phase_name)
            if current_distance_mm is None:
                return False

            actual_segment_mm = current_distance_mm - confirmed_distance_mm
            segment_error_mm = actual_segment_mm - segment_distance_mm
            if abs(segment_error_mm) <= self.forward_stop_tolerance_mm:
                rospy.loginfo(
                    f"{phase_name}: 实际本段位移={actual_segment_mm:.1f} mm, "
                    f"目标本段位移={segment_distance_mm:.1f} mm, "
                    f"本段误差={segment_error_mm:+.1f} mm, 已在容差内",
                )
            else:
                rospy.logwarn(
                    f"{phase_name}: 实际本段位移={actual_segment_mm:.1f} mm, "
                    f"目标本段位移={segment_distance_mm:.1f} mm, "
                    f"本段误差={segment_error_mm:+.1f} mm, "
                    f"超出容差={self.forward_stop_tolerance_mm:.1f} mm",
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

    def _run_forward_phase_with_x_targets(self):
        target_x_offsets_mm = self._build_forward_target_x_offsets_mm()
        if not target_x_offsets_mm:
            rospy.loginfo("累计目标X序列为空，跳过前进")
            return True

        rospy.loginfo(
            "本轮累计目标X概要: "
            + ", ".join(f"{value / 1000.0:.3f}m" for value in target_x_offsets_mm)
        )

        if not self.send_harvest_status(False):
            return False

        last_confirmed_x_offset_mm = 0.0
        for index, target_x_offset_mm in enumerate(target_x_offsets_mm, start=1):
            phase_name = f"前进分段{index}"
            current_x_offset_mm = self._get_fast_x_offset_from_start_mm()
            if current_x_offset_mm is None:
                current_x_offset_mm = last_confirmed_x_offset_mm

            rospy.loginfo(
                f"{phase_name}: 当前累计X偏移={current_x_offset_mm:.1f} mm, "
                f"上一确认累计X偏移={last_confirmed_x_offset_mm:.1f} mm, "
                f"本段累计目标X偏移={target_x_offset_mm:.1f} mm, "
                f"本段计划增量={(target_x_offset_mm - last_confirmed_x_offset_mm):+.1f} mm",
            )

            current_x_offset_mm = self._move_to_target_x_offset_mm(target_x_offset_mm, phase_name)
            if current_x_offset_mm is None:
                return False

            actual_segment_mm = current_x_offset_mm - last_confirmed_x_offset_mm
            planned_segment_mm = target_x_offset_mm - last_confirmed_x_offset_mm
            segment_error_mm = actual_segment_mm - planned_segment_mm
            if abs(segment_error_mm) <= self.forward_stop_tolerance_mm:
                rospy.loginfo(
                    f"{phase_name}: 实际本段增量={actual_segment_mm:.1f} mm, "
                    f"目标本段增量={planned_segment_mm:.1f} mm, "
                    f"本段误差={segment_error_mm:+.1f} mm, 已在容差内",
                )
            else:
                rospy.logwarn(
                    f"{phase_name}: 实际本段增量={actual_segment_mm:.1f} mm, "
                    f"目标本段增量={planned_segment_mm:.1f} mm, "
                    f"本段误差={segment_error_mm:+.1f} mm, "
                    f"超出容差={self.forward_stop_tolerance_mm:.1f} mm",
                )
            last_confirmed_x_offset_mm = current_x_offset_mm

            if not self.wait_for_harvest_rearm_window(phase_name):
                return False
            if not self.send_harvest_status(True):
                return False
            if not self.wait_for_harvest_complete(phase_name):
                return False

            if index < len(target_x_offsets_mm):
                rospy.loginfo(
                    f"{phase_name}: 采摘完成，继续前进到下一累计X目标 "
                    f"{target_x_offsets_mm[index]:.1f} mm"
                )
                if not self.send_harvest_status(False):
                    return False

        if not self.send_harvest_status(False):
            return False

        self.record_forward_end_odometry()
        return True


def main():
    signal.signal(signal.SIGINT, signal_handler)
    controller = UpRail416Controller()
    controller.run()


if __name__ == "__main__":
    main()
