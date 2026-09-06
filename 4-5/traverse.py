#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于 Flag_4_2 的导航遍历控制器。

职责:
1. 默认等待导航状态话题进入 SUCCEEDED
2. 收到导航到位后放开对轨节点(new_copy_no_capture 的门控)
3. 收到对轨完成触发 /flag1 后执行上轨/下轨
4. 后续由 Flag_4_2 的横移接管遍历
5. 也兼容旧模式: 由 traverse 主动发布导航点

说明:
- 上/下轨控制沿用 Flag_4_2 的原逻辑
- 支持两种模式:
  a) 只导航首个点一次，后续靠横移遍历
  b) 每个点都重新导航
"""

import math
import os
import threading
import time

import rospy
import rosgraph
import rosparam
from geometry_msgs.msg import PoseStamped
from rover_msgs.msg import roverGoalStatus
from std_msgs.msg import Int32

from Flag_4_2 import UpRailFlagController


def wait_for_ros_master(timeout_sec=60.0):
    deadline = time.time() + max(0.0, float(timeout_sec))
    master = rosgraph.Master("/traverse_yaml_loader")
    while not rospy.is_shutdown():
        try:
            master.getPid()
            return True
        except Exception:
            if time.time() >= deadline:
                return False
            time.sleep(0.2)
    return False


def auto_load_workflow_yaml():
    workflow_yaml = os.environ.get(
        "RAIL_WORKFLOW_YAML",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "traverse.yaml"),
    )
    if not os.path.exists(workflow_yaml):
        return
    wait_timeout_sec = float(os.environ.get("RAIL_WORKFLOW_MASTER_WAIT_SEC", "60"))
    if not wait_for_ros_master(wait_timeout_sec):
        print(
            f"[traverse] warning: ROS master not available within {wait_timeout_sec:.1f}s; "
            f"workflow yaml not loaded: {workflow_yaml}"
        )
        return
    try:
        param_list = rosparam.load_file(workflow_yaml)
        for params, namespace in param_list:
            rosparam.upload_params(namespace, params)
        print(f"[traverse] auto-loaded workflow yaml: {workflow_yaml}")
    except Exception as exc:
        print(f"[traverse] warning: failed to auto-load workflow yaml: {workflow_yaml}, error={exc}")


class TraverseController(UpRailFlagController):
    def __init__(self):
        os.environ.setdefault("RAIL_CONTROLLER_NODE_NAME", "traverse_rail_controller")
        os.environ.setdefault("RAIL_CONTROLLER_STARTUP_TITLE", "traverse 轨道遍历控制器底层动作模块已启动")
        super().__init__()

        self.use_external_heading_reference = bool(
            rospy.get_param(
                f"{self.workflow_param_ns}/use_external_heading_reference",
                self.use_external_heading_reference,
            )
        )
        self.capture_initial_heading_before_first_cycle = bool(
            rospy.get_param(
                f"{self.workflow_param_ns}/capture_initial_heading_before_first_cycle",
                self.capture_initial_heading_before_first_cycle,
            )
        )
        if (
            not self.capture_initial_heading_before_first_cycle
            and self.heading_reference_source == "startup_fast"
        ):
            self.heading_reference_deg = None
            self.heading_reference_source = None
            self.heading_reference_capture_wall_time = 0.0
            rospy.loginfo("traverse 已清除启动阶段锁存的 FAST 基准角，后续改为使用导航成功时刻的角度")

        self.nav_goal_topic = rospy.get_param(
            "~nav_goal_topic",
            rospy.get_param(f"{self.workflow_param_ns}/nav_goal_topic", "/cur_goal"),
        )
        self.nav_status_topic = rospy.get_param(
            "~nav_status_topic",
            rospy.get_param(f"{self.workflow_param_ns}/nav_status_topic", "/cur_local_goal_status"),
        )
        self.nav_frame_id = rospy.get_param(
            "~nav_frame_id",
            rospy.get_param(f"{self.workflow_param_ns}/nav_frame_id", "camera_init"),
        )
        self.nav_goal_z = float(
            rospy.get_param(
                "~nav_goal_z",
                rospy.get_param(f"{self.workflow_param_ns}/nav_goal_z", 0.0),
            )
        )
        self.nav_goal_match_tolerance_m = float(
            rospy.get_param(
                "~nav_goal_match_tolerance_m",
                rospy.get_param(f"{self.workflow_param_ns}/nav_goal_match_tolerance_m", 2.0),
            )
        )
        self.nav_goal_publish_repeat_count = int(
            rospy.get_param(
                "~nav_goal_publish_repeat_count",
                rospy.get_param(f"{self.workflow_param_ns}/nav_goal_publish_repeat_count", 2),
            )
        )
        self.nav_goal_publish_repeat_interval_sec = float(
            rospy.get_param(
                "~nav_goal_publish_repeat_interval_sec",
                rospy.get_param(
                    f"{self.workflow_param_ns}/nav_goal_publish_repeat_interval_sec",
                    0.15,
                ),
            )
        )
        self.nav_goal_publish_wait_connections_sec = float(
            rospy.get_param(
                "~nav_goal_publish_wait_connections_sec",
                rospy.get_param(
                    f"{self.workflow_param_ns}/nav_goal_publish_wait_connections_sec",
                    1.0,
                ),
            )
        )
        self.nav_arrive_settle_sec = float(
            rospy.get_param(
                "~nav_arrive_settle_sec",
                rospy.get_param(f"{self.workflow_param_ns}/nav_arrive_settle_sec", 0.8),
            )
        )
        self.restore_heading_before_alignment = bool(
            rospy.get_param(
                "~restore_heading_before_alignment",
                rospy.get_param(
                    f"{self.workflow_param_ns}/restore_heading_before_alignment",
                    True,
                ),
            )
        )
        self.nav_goal_timeout_sec = float(
            rospy.get_param(
                "~nav_goal_timeout_sec",
                rospy.get_param(f"{self.workflow_param_ns}/nav_goal_timeout_sec", 0.0),
            )
        )
        self.nav_goal_retry_count = int(
            rospy.get_param(
                "~nav_goal_retry_count",
                rospy.get_param(f"{self.workflow_param_ns}/nav_goal_retry_count", 2),
            )
        )
        self.alignment_enable_topic = rospy.get_param(
            "~alignment_enable_topic",
            rospy.get_param(
                f"{self.workflow_param_ns}/alignment_enable_topic",
                "/traverse/alignment_enabled",
            ),
        )
        self.wait_nav_ready_only = bool(
            rospy.get_param(
                "~wait_nav_ready_only",
                rospy.get_param(
                    f"{self.workflow_param_ns}/wait_nav_ready_only",
                    True,
                ),
            )
        )
        self.nav_success_value = int(
            rospy.get_param(
                "~nav_success_value",
                rospy.get_param(
                    f"{self.workflow_param_ns}/nav_success_value",
                    roverGoalStatus.SUCCEEDED,
                ),
            )
        )
        self.nav_ready_topic = rospy.get_param(
            "~nav_ready_topic",
            rospy.get_param(
                f"{self.workflow_param_ns}/nav_ready_topic",
                "/nav_ready_flag",
            ),
        )
        self.enable_nav_ready_fallback = bool(
            rospy.get_param(
                "~enable_nav_ready_fallback",
                rospy.get_param(
                    f"{self.workflow_param_ns}/enable_nav_ready_fallback",
                    False,
                ),
            )
        )
        self.nav_ready_value = int(
            rospy.get_param(
                "~nav_ready_value",
                rospy.get_param(
                    f"{self.workflow_param_ns}/nav_ready_value",
                    1,
                ),
            )
        )
        self.navigate_first_goal_only = bool(
            rospy.get_param(
                "~navigate_first_goal_only",
                rospy.get_param(
                    f"{self.workflow_param_ns}/navigate_first_goal_only",
                    True,
                ),
            )
        )
        self.enable_post_return_lateral_in_traverse = bool(
            rospy.get_param(
                "~enable_post_return_lateral_in_traverse",
                rospy.get_param(
                    f"{self.workflow_param_ns}/enable_post_return_lateral_in_traverse",
                    False,
                ),
            )
        )
        self.use_direct_forward_distance_table = bool(
            rospy.get_param(
                "~use_direct_forward_distance_table",
                rospy.get_param(
                    f"{self.workflow_param_ns}/use_direct_forward_distance_table",
                    True,
                ),
            )
        )
        self.forward_distances_mm = self._load_float_list_param(
            "~forward_distances_mm",
            f"{self.workflow_param_ns}/forward_distances_mm",
            "上轨前进距离",
        )
        self.forward_direct_timeout_sec = float(
            rospy.get_param(
                "~forward_direct_timeout_sec",
                rospy.get_param(f"{self.workflow_param_ns}/forward_direct_timeout_sec", 0.0),
            )
        )
        self.forward_direct_timeout_margin_sec = float(
            rospy.get_param(
                "~forward_direct_timeout_margin_sec",
                rospy.get_param(f"{self.workflow_param_ns}/forward_direct_timeout_margin_sec", 8.0),
            )
        )
        self.forward_direct_timeout_min_sec = float(
            rospy.get_param(
                "~forward_direct_timeout_min_sec",
                rospy.get_param(f"{self.workflow_param_ns}/forward_direct_timeout_min_sec", 15.0),
            )
        )
        self.logged_forward_distance_fallback_indices = set()
        self.use_first_aligned_x_return = bool(
            rospy.get_param(
                "~use_first_aligned_x_return",
                rospy.get_param(
                    f"{self.workflow_param_ns}/use_first_aligned_x_return",
                    True,
                ),
            )
        )
        self.first_aligned_return_x_m = None
        self.first_aligned_return_x_sequence = None
        self.use_absolute_x_return_check = bool(
            rospy.get_param(
                "~use_absolute_x_return_check",
                rospy.get_param(
                    f"{self.workflow_param_ns}/use_absolute_x_return_check",
                    False,
                ),
            )
        )
        self.absolute_x_return_reference_source = str(
            rospy.get_param(
                "~absolute_x_return_reference_source",
                rospy.get_param(
                    f"{self.workflow_param_ns}/absolute_x_return_reference_source",
                    "navigation_success",
                ),
            )
        ).strip().lower()
        self.rail_reference_points = self._load_rail_reference_points()
        self.rail_reference_by_id = {
            int(point["rail"]): point for point in self.rail_reference_points
        }
        self.rail_traverse_sequence = self._load_rail_traverse_sequence()
        self.active_return_target_x_m = None
        self.active_return_target_source = None

        self.navigation_goals = self._load_navigation_goals()
        self.goal_pub = rospy.Publisher(self.nav_goal_topic, PoseStamped, queue_size=1, latch=True)
        self.alignment_enable_pub = rospy.Publisher(
            self.alignment_enable_topic, Int32, queue_size=1, latch=True
        )
        self.nav_status_sub = rospy.Subscriber(
            self.nav_status_topic,
            roverGoalStatus,
            self.nav_status_callback,
            queue_size=20,
        )
        self.nav_ready_sub = None
        if self.enable_nav_ready_fallback:
            self.nav_ready_sub = rospy.Subscriber(
                self.nav_ready_topic,
                Int32,
                self.nav_ready_callback,
                queue_size=8,
            )

        self.nav_lock = threading.Lock()
        self.nav_current_goal_index = -1
        self.nav_current_goal = None
        self.nav_goal_sent_wall_time = 0.0
        self.nav_current_retry = 0
        self.nav_seen_active = False
        self.nav_waiting_for_alignment = False
        self.nav_finished = False
        self.nav_failed = False
        self.nav_last_status = roverGoalStatus.PENDING
        self.nav_last_text = "INIT"
        self.nav_ready_received = False
        self.nav_success_consumed = False
        self.navigation_heading_locked = False
        self.navigation_success_position = None

        self.publish_alignment_enable(False, "启动默认关闭对轨")

        rospy.loginfo("=" * 70)
        rospy.loginfo("traverse 轨道遍历控制器已启动")
        rospy.loginfo(f"  导航目标话题: {self.nav_goal_topic}")
        rospy.loginfo(f"  导航状态话题: {self.nav_status_topic}")
        rospy.loginfo(f"  对轨门控话题: {self.alignment_enable_topic}")
        rospy.loginfo(f"  导航点数量: {len(self.navigation_goals)}")
        if self.navigation_goals:
            labels = ", ".join(goal["label"] for goal in self.navigation_goals)
            rospy.loginfo(f"  遍历顺序: {labels}")
        rospy.loginfo(
            f"  启动方式: {'等待导航到位标志位' if self.wait_nav_ready_only else ('只导航首轨一次，后续靠横移遍历' if self.navigate_first_goal_only else '每个点都重新导航')}"
        )
        if self.wait_nav_ready_only:
            rospy.loginfo(
                f"  导航到位状态: topic={self.nav_status_topic}, success_value={self.nav_success_value}"
            )
        rospy.loginfo(
            f"  nav_ready 兼容触发: "
            f"{'启用' if self.enable_nav_ready_fallback else '禁用'}"
        )
        rospy.loginfo(
            f"  导航到位后先回导航成功基准角: "
            f"{'启用' if self.restore_heading_before_alignment else '禁用'}"
        )
        rospy.loginfo(
            f"  回零后横移: {'启用' if self.enable_post_return_lateral_in_traverse else '关闭(由导航接管点间切换)'}"
        )
        rospy.loginfo(
            f"  traverse 直行上轨距离表: "
            f"{'启用' if self.use_direct_forward_distance_table else '禁用'}"
        )
        if self.forward_distances_mm:
            rospy.loginfo(f"  上轨前进距离列表(mm): {self.forward_distances_mm}")
        rospy.loginfo(
            f"  回退目标x锁定: {'启用' if self.use_first_aligned_x_return else '禁用'}"
        )
        rospy.loginfo(
            f"  下轨回退x来源: "
            f"{'启用下一条轨道实测x' if self.use_absolute_x_return_check else '沿用原始回零目标'}, "
            f"source={self.absolute_x_return_reference_source}, "
            f"rail_sequence={self.rail_traverse_sequence}"
        )
        rospy.loginfo("=" * 70)

    def _load_float_list_param(self, private_key, global_key, label):
        if rospy.has_param(private_key):
            raw_value = rospy.get_param(private_key)
        elif rospy.has_param(global_key):
            raw_value = rospy.get_param(global_key)
        else:
            return []

        if isinstance(raw_value, (list, tuple)):
            values = []
            for idx, item in enumerate(raw_value, start=1):
                try:
                    values.append(float(item))
                except Exception:
                    rospy.logwarn(f"{label}列表第{idx}项无法解析为数字: {item}")
            return values

        try:
            return [float(raw_value)]
        except Exception:
            rospy.logwarn(
                f"{label}参数解析失败: private={private_key}, global={global_key}, value={raw_value}"
            )
            return []

    def _load_int_list_param(self, private_key, global_key, label):
        if rospy.has_param(private_key):
            raw_value = rospy.get_param(private_key)
        elif rospy.has_param(global_key):
            raw_value = rospy.get_param(global_key)
        else:
            return []

        if isinstance(raw_value, (list, tuple)):
            values = []
            for idx, item in enumerate(raw_value, start=1):
                try:
                    values.append(int(item))
                except Exception:
                    rospy.logwarn(f"{label}列表第{idx}项无法解析为整数: {item}")
            return values

        try:
            return [int(raw_value)]
        except Exception:
            rospy.logwarn(
                f"{label}参数解析失败: private={private_key}, global={global_key}, value={raw_value}"
            )
            return []

    def _load_rail_reference_points(self):
        raw_value = rospy.get_param("~rail_reference_points", None)
        if raw_value is None:
            raw_value = rospy.get_param(f"{self.workflow_param_ns}/rail_reference_points", [])
        if raw_value is None:
            return []
        if not isinstance(raw_value, list):
            rospy.logwarn(
                f"rail_reference_points 参数格式不支持: {type(raw_value)}，应为列表，已忽略"
            )
            return []

        points = []
        for idx, item in enumerate(raw_value, start=1):
            if not isinstance(item, dict):
                rospy.logwarn(f"rail_reference_points[{idx}] 不是字典，已跳过: {item}")
                continue
            try:
                rail_id = int(item.get("rail", item.get("id", idx)))
                x_m = float(item.get("x_m", item.get("x")))
                y_m = float(item.get("y_m", item.get("y")))
            except Exception:
                rospy.logwarn(f"rail_reference_points[{idx}] 缺少 rail/x_m/y_m，已跳过: {item}")
                continue
            points.append(
                {
                    "rail": rail_id,
                    "x_m": x_m,
                    "y_m": y_m,
                    "label": str(item.get("label", f"rail_{rail_id}")),
                }
            )
        return points

    def _load_rail_traverse_sequence(self):
        sequence = self._load_int_list_param(
            "~rail_traverse_sequence",
            f"{self.workflow_param_ns}/rail_traverse_sequence",
            "轨道遍历顺序",
        )
        if sequence:
            return sequence
        if not self.rail_reference_points:
            return []
        return [int(point["rail"]) for point in self.rail_reference_points]

    def _load_navigation_goals(self):
        raw_value = rospy.get_param("~navigation_goals", None)
        if raw_value is None:
            raw_value = rospy.get_param(f"{self.workflow_param_ns}/navigation_goals", [])
        if raw_value is not None and not isinstance(raw_value, list):
            rospy.logwarn(
                f"navigation_goals 参数格式不支持: {type(raw_value)}，应为列表，已忽略"
            )
            raw_value = []

        goals = self._normalize_navigation_goals(raw_value)
        if goals:
            return goals
        if self.wait_nav_ready_only:
            return []

        return self._generate_navigation_goals_from_pattern()

    def _normalize_navigation_goals(self, raw_goals):
        if raw_goals is None:
            return []

        goals = []
        for index, item in enumerate(raw_goals, start=1):
            if not isinstance(item, dict):
                rospy.logwarn(f"navigation_goals[{index}] 不是字典，已跳过: {item}")
                continue
            try:
                x = float(item["x"])
                y = float(item["y"])
            except Exception:
                rospy.logwarn(f"navigation_goals[{index}] 缺少 x/y，已跳过: {item}")
                continue
            yaw_deg = float(item.get("yaw_deg", item.get("yaw", 0.0)))
            label = str(item.get("label", item.get("name", f"point_{index}")))
            frame_id = str(item.get("frame_id", self.nav_frame_id))
            z = float(item.get("z", self.nav_goal_z))
            goals.append(
                {
                    "index": index,
                    "label": label,
                    "frame_id": frame_id,
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw_deg": yaw_deg,
                }
            )
        return goals

    def _generate_navigation_goals_from_pattern(self):
        first_goal = rospy.get_param("~first_goal", None)
        if first_goal is None:
            first_goal = rospy.get_param(f"{self.workflow_param_ns}/first_goal", None)
        if not isinstance(first_goal, dict):
            return []

        rail_count = int(
            rospy.get_param(
                "~rail_count",
                rospy.get_param(
                    f"{self.workflow_param_ns}/rail_count",
                    rospy.get_param(f"{self.workflow_param_ns}/cycle_count", 0),
                ),
            )
        )
        rail_spacing_mm = float(
            rospy.get_param(
                "~rail_spacing_mm",
                rospy.get_param(
                    f"{self.workflow_param_ns}/rail_spacing_mm",
                    rospy.get_param(f"{self.workflow_param_ns}/rail_spacing_m", 1.7) * 1000.0,
                ),
            )
        )
        rail_spacing_m = rail_spacing_mm / 1000.0
        rail_axis = str(
            rospy.get_param(
                "~rail_axis",
                rospy.get_param(f"{self.workflow_param_ns}/rail_axis", "x"),
            )
        ).strip().lower()
        rail_direction_sign = float(
            rospy.get_param(
                "~rail_direction_sign",
                rospy.get_param(f"{self.workflow_param_ns}/rail_direction_sign", 1.0),
            )
        )
        raw_spacing_list_mm = rospy.get_param("~rail_spacing_list_mm", None)
        if raw_spacing_list_mm is None:
            raw_spacing_list_mm = rospy.get_param(
                f"{self.workflow_param_ns}/rail_spacing_list_mm",
                [],
            )
        spacing_list_m = []
        if isinstance(raw_spacing_list_mm, list):
            for item in raw_spacing_list_mm:
                try:
                    spacing_list_m.append(float(item) / 1000.0)
                except Exception:
                    rospy.logwarn(f"rail_spacing_list_mm 存在非法值，已跳过: {item}")
        elif raw_spacing_list_mm not in (None, []):
            rospy.logwarn(
                f"rail_spacing_list_mm 参数格式不支持: {type(raw_spacing_list_mm)}，应为列表，已忽略"
            )

        if rail_count <= 0:
            rospy.logwarn("未生成 navigation_goals: rail_count<=0")
            return []
        if rail_axis not in ("x", "y"):
            rospy.logwarn(f"未生成 navigation_goals: rail_axis={rail_axis} 非法，只支持 x/y")
            return []

        try:
            base_x = float(first_goal["x"])
            base_y = float(first_goal["y"])
        except Exception:
            rospy.logwarn(f"未生成 navigation_goals: first_goal 缺少 x/y: {first_goal}")
            return []

        base_yaw_deg = float(first_goal.get("yaw_deg", first_goal.get("yaw", 0.0)))
        base_label = str(first_goal.get("label", "point"))
        base_frame_id = str(first_goal.get("frame_id", self.nav_frame_id))
        base_z = float(first_goal.get("z", self.nav_goal_z))

        effective_rail_count = 1 if self.navigate_first_goal_only else rail_count
        goals = []
        offset_m = 0.0
        for index in range(effective_rail_count):
            x = base_x
            y = base_y
            if rail_axis == "x":
                x += offset_m
            else:
                y += offset_m
            goals.append(
                {
                    "index": index + 1,
                    "label": f"{base_label}_{index + 1}",
                    "frame_id": base_frame_id,
                    "x": x,
                    "y": y,
                    "z": base_z,
                    "yaw_deg": base_yaw_deg,
                }
            )
            if index < effective_rail_count - 1:
                if index < len(spacing_list_m):
                    gap_m = spacing_list_m[index]
                else:
                    gap_m = rail_spacing_m
                offset_m += gap_m * rail_direction_sign

        rospy.loginfo(
            "未提供显式 navigation_goals，已按 first_goal 自动生成 %d 个导航点: "
            "axis=%s, default_spacing=%.3f m, spacing_list_len=%d, sign=%+.1f",
            effective_rail_count,
            rail_axis,
            rail_spacing_m,
            len(spacing_list_m),
            rail_direction_sign,
        )
        return goals

    def should_run_post_return_lateral_move(self, sequence_id):
        if self.enable_post_return_lateral_in_traverse:
            return super().should_run_post_return_lateral_move(sequence_id)
        return False

    def publish_alignment_enable(self, enabled, reason):
        value = 1 if enabled else 0
        try:
            self.alignment_enable_pub.publish(Int32(data=value))
            state_text = "放开对轨" if enabled else "关闭对轨"
            rospy.loginfo(f"{state_text}: value={value}, reason={reason}")
        except Exception as exc:
            rospy.logwarn(f"发布对轨门控失败: enabled={enabled}, error={exc}")

    @staticmethod
    def _yaw_deg_to_quaternion(yaw_deg):
        yaw_rad = math.radians(float(yaw_deg))
        half = 0.5 * yaw_rad
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }

    def _status_matches_goal(self, msg, goal):
        dx = float(msg.x) - float(goal["x"])
        dy = float(msg.y) - float(goal["y"])
        dist = math.hypot(dx, dy)
        return dist <= max(0.05, self.nav_goal_match_tolerance_m)

    def _publish_current_goal(self, reason):
        goal = self.nav_current_goal
        if goal is None:
            return False

        deadline = time.monotonic() + max(0.0, self.nav_goal_publish_wait_connections_sec)
        while self.goal_pub.get_num_connections() == 0 and time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.05)

        quat = self._yaw_deg_to_quaternion(goal["yaw_deg"])
        msg = PoseStamped()
        msg.header.frame_id = goal["frame_id"]
        msg.pose.position.x = goal["x"]
        msg.pose.position.y = goal["y"]
        msg.pose.position.z = goal["z"]
        msg.pose.orientation.x = quat["x"]
        msg.pose.orientation.y = quat["y"]
        msg.pose.orientation.z = quat["z"]
        msg.pose.orientation.w = quat["w"]

        publish_times = max(1, self.nav_goal_publish_repeat_count)
        for publish_index in range(publish_times):
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            rospy.loginfo(
                "发布导航点[%d/%d] %s -> x=%.3f, y=%.3f, yaw=%.1f deg (%d/%d, reason=%s, subscribers=%d)",
                self.nav_current_goal_index + 1,
                len(self.navigation_goals),
                goal["label"],
                goal["x"],
                goal["y"],
                goal["yaw_deg"],
                publish_index + 1,
                publish_times,
                reason,
                self.goal_pub.get_num_connections(),
            )
            if publish_index + 1 < publish_times:
                rospy.sleep(max(0.0, self.nav_goal_publish_repeat_interval_sec))

        self.nav_goal_sent_wall_time = time.time()
        self.nav_seen_active = False
        self.nav_waiting_for_alignment = False
        self.nav_last_status = roverGoalStatus.PENDING
        self.nav_last_text = "PENDING"
        with self.trigger_lock:
            self.trigger_requested = False
        return True

    def _start_navigation_goal(self, goal_index, reason):
        if goal_index < 0 or goal_index >= len(self.navigation_goals):
            return False

        with self.nav_lock:
            self.nav_current_goal_index = goal_index
            self.nav_current_goal = self.navigation_goals[goal_index]
            self.nav_current_retry = 0
            self.nav_finished = False
            self.nav_failed = False
            self.nav_success_consumed = False

        self.publish_alignment_enable(False, f"开始导航 {self.nav_current_goal['label']}")
        return self._publish_current_goal(reason)

    def _advance_to_next_goal(self, reason):
        next_index = self.nav_current_goal_index + 1
        if next_index >= len(self.navigation_goals):
            with self.nav_lock:
                self.nav_finished = True
                self.nav_current_goal = None
            self.publish_alignment_enable(False, "所有导航点已完成")
            rospy.loginfo("所有导航点遍历完成")
            return False
        return self._start_navigation_goal(next_index, reason)

    def _handle_navigation_failure(self, status_value, text):
        with self.nav_lock:
            goal = self.nav_current_goal
            retry = self.nav_current_retry
        if goal is None:
            return

        if retry < self.nav_goal_retry_count:
            with self.nav_lock:
                self.nav_current_retry += 1
                retry = self.nav_current_retry
            rospy.logwarn(
                f"导航点 {goal['label']} 状态={status_value}({text})，准备重发 "
                f"{retry}/{self.nav_goal_retry_count}"
            )
            self._publish_current_goal(f"retry_{retry}")
            return

        with self.nav_lock:
            self.nav_failed = True
        self.publish_alignment_enable(False, f"导航失败: {goal['label']}")
        rospy.logerr(
            f"导航点 {goal['label']} 连续失败，状态={status_value}({text})，遍历终止"
        )

    def nav_ready_callback(self, msg):
        if int(msg.data) != self.nav_ready_value:
            return

        with self.nav_lock:
            if self.nav_failed or self.nav_finished:
                return
            already_waiting = self.nav_waiting_for_alignment
            success_consumed = self.nav_success_consumed
            self.nav_ready_received = True
            if not success_consumed:
                self.nav_waiting_for_alignment = True
                self.nav_success_consumed = True

        if already_waiting or success_consumed:
            return

        rospy.loginfo(
            f"收到导航到位标志位: topic={self.nav_ready_topic}, value={int(msg.data)}，准备先回导航成功基准角再对轨"
        )
        self._prepare_alignment_after_nav_success("收到导航到位标志位")

    def _has_more_cycles_after(self, completed_sequence_id):
        planned = int(self.planned_cycle_count)
        if planned == 0:
            return True
        if planned < 0:
            return False
        return int(completed_sequence_id) < planned

    def _open_alignment_for_next_cycle(self, reason):
        with self.nav_lock:
            self.nav_waiting_for_alignment = True
        self.switch_mode_if_needed(self.MODE_LATERAL)
        self.publish_alignment_enable(True, reason)

    def get_post_return_lateral_distance_mm(self, sequence_id):
        distance_mm = super().get_post_return_lateral_distance_mm(sequence_id)
        if distance_mm is None:
            return None
        return float(distance_mm)

    def _get_rail_reference_for_sequence_index(self, sequence_index):
        index = int(sequence_index)
        if index < 0 or index >= len(self.rail_traverse_sequence):
            return None
        rail_id = int(self.rail_traverse_sequence[index])
        return self.rail_reference_by_id.get(rail_id)

    def _get_current_rail_reference(self, sequence_id):
        return self._get_rail_reference_for_sequence_index(int(sequence_id) - 1)

    def _get_next_rail_reference(self, sequence_id):
        return self._get_rail_reference_for_sequence_index(int(sequence_id))

    def _get_next_rail_return_x_m(self, sequence_id):
        if not self.use_absolute_x_return_check:
            return None, "disabled"

        source = self.absolute_x_return_reference_source
        if source not in ("rail_reference_next", "next_rail", "rail_next", "measured_next"):
            return None, source

        current_ref = self._get_current_rail_reference(sequence_id)
        next_ref = self._get_next_rail_reference(sequence_id)
        if next_ref is None:
            return None, "rail_reference_next_last_or_missing"

        current_label = "unknown" if current_ref is None else current_ref["label"]
        return (
            float(next_ref["x_m"]),
            f"rail_reference_next:{current_label}->{next_ref['label']}",
        )

    def get_signed_fast_offset_mm(self):
        signed_offset_mm, heading_snapshot, axis = super().get_signed_fast_offset_mm()
        if axis is None or heading_snapshot is None:
            return signed_offset_mm, heading_snapshot, axis

        target_x_m = self.active_return_target_x_m
        if target_x_m is None:
            return signed_offset_mm, heading_snapshot, axis

        position = heading_snapshot.get("position")
        if position is None:
            return signed_offset_mm, heading_snapshot, axis

        current_x_m = float(position[0])
        target_x_m = float(target_x_m)
        axis["current_x_m"] = current_x_m
        axis["target_x_m"] = target_x_m
        axis["x_error_mm"] = (current_x_m - target_x_m) * 1000.0
        axis["x_target_source"] = self.active_return_target_source
        return signed_offset_mm, heading_snapshot, axis

    def get_forward_distance_mm(self, sequence_id):
        forward_index = int(sequence_id)
        if len(self.forward_distances_mm) == 0:
            return max(0.0, float(self.forward_distance_mm))

        if forward_index <= len(self.forward_distances_mm):
            return max(0.0, float(self.forward_distances_mm[forward_index - 1]))

        fallback_distance = max(0.0, float(self.forward_distances_mm[-1]))
        if forward_index not in self.logged_forward_distance_fallback_indices:
            self.logged_forward_distance_fallback_indices.add(forward_index)
            rospy.logwarn(
                f"第{forward_index}轮未配置独立上轨前进距离，复用最后一个距离 "
                f"{fallback_distance:.1f} mm"
            )
        return fallback_distance

    def _estimate_direct_forward_timeout_sec(self, distance_mm):
        if self.forward_direct_timeout_sec <= 0.0:
            return 0.0
        return float(self.forward_direct_timeout_sec)

    def _update_first_aligned_return_x(self, sequence_id):
        if not self.use_first_aligned_x_return:
            return
        if self.start_fast_position is None:
            return

        current_x_m = float(self.start_fast_position[0])
        if self.first_aligned_return_x_m is None:
            self.first_aligned_return_x_m = current_x_m
            self.first_aligned_return_x_sequence = int(sequence_id)
            rospy.loginfo(
                f"第{sequence_id}轮对轨后锁定首次回退x: "
                f"x={self.first_aligned_return_x_m:+.3f} m，后续所有轨道回退都回到这个x"
            )
            return

        delta_x_mm = (current_x_m - float(self.first_aligned_return_x_m)) * 1000.0
        rospy.loginfo(
            f"第{sequence_id}轮对轨后起点x={current_x_m:+.3f} m，"
            f"沿用第{self.first_aligned_return_x_sequence}轮首次x="
            f"{float(self.first_aligned_return_x_m):+.3f} m，"
            f"本轮回退将修正x差={delta_x_mm:+.1f} mm"
        )

    def run_forward_phase(self):
        if not self.use_direct_forward_distance_table:
            return super().run_forward_phase()

        sequence_id = int(self.sequence_counter)
        total_target_mm = self.get_forward_distance_mm(sequence_id)
        phase_name = f"第{sequence_id}轮直行上轨"

        rospy.loginfo("=" * 60)
        rospy.loginfo(
            f"阶段1: traverse 按 yaml 距离表直接上轨，不分段、不等待 021，"
            f"target={total_target_mm:.1f} mm"
        )
        rospy.loginfo("=" * 60)

        self.record_start_odometry()
        if self.start_fast_position is None:
            rospy.logerr("前进阶段失败: 未记录到有效起点 FAST 位姿")
            return False
        self._update_first_aligned_return_x(sequence_id)

        if total_target_mm <= 0.0:
            rospy.loginfo(f"{phase_name}: 目标距离为 0，跳过上轨前进")
            self.record_forward_end_odometry()
            return True

        if not self.send_speed_mode_command(self.forward_linear_speed_mm_s, self.angular_speed_cmd):
            return False

        timeout_sec = self._estimate_direct_forward_timeout_sec(total_target_mm)
        if timeout_sec <= 0.0:
            rospy.loginfo(f"{phase_name}: 已禁用时间超时，只按 FAST 距离到达目标后停车")
        else:
            rospy.loginfo(f"{phase_name}: 启用时间超时 timeout={timeout_sec:.1f}s")
        start_wall_time = time.time()
        last_distance_mm = 0.0

        while self.running and not rospy.is_shutdown():
            fast_distance_mm, _ = self.get_distance_to_fast_start_mm()

            if fast_distance_mm is not None:
                last_distance_mm = max(0.0, abs(float(fast_distance_mm)))
                rospy.loginfo_throttle(
                    0.5,
                    f"{phase_name}: fast_dist_mm={last_distance_mm:.1f}, "
                    f"target_mm={total_target_mm:.1f}"
                )
                if last_distance_mm >= total_target_mm:
                    rospy.loginfo(
                        f"{phase_name}: 到达 yaml 目标距离 fast_dist_mm={last_distance_mm:.1f}, "
                        f"target_mm={total_target_mm:.1f}"
                    )
                    break
            else:
                rospy.logwarn_throttle(
                    1.0,
                    f"{phase_name}: 暂时无法读取 FAST 距离，继续等待 {self.heading_topic}"
                )

            if timeout_sec > 0.0 and time.time() - start_wall_time > timeout_sec:
                rospy.logerr(
                    f"{phase_name}: 前进超时 timeout={timeout_sec:.1f}s, "
                    f"last_fast_dist_mm={last_distance_mm:.1f}, target_mm={total_target_mm:.1f}"
                )
                self.stop_speed_motion()
                self.wait_until_speed_zero(phase_name=f"{phase_name}超时停车")
                time.sleep(self.stop_settle_sec)
                return False

            time.sleep(0.02)

        self.stop_speed_motion()
        self.wait_until_speed_zero(phase_name=f"{phase_name}停车")
        time.sleep(self.stop_settle_sec)

        self.record_forward_end_odometry()
        return True

    def run_reverse_phase(self):
        original_start_fast_position = self.start_fast_position
        target_x_m, target_source = self._get_next_rail_return_x_m(int(self.sequence_counter))
        if target_x_m is not None and original_start_fast_position is not None:
            previous_target_x_m = self.active_return_target_x_m
            previous_target_source = self.active_return_target_source
            target_start_position = (
                float(target_x_m),
                float(original_start_fast_position[1]),
                float(original_start_fast_position[2]),
            )
            delta_x_mm = (float(original_start_fast_position[0]) - float(target_x_m)) * 1000.0
            rospy.loginfo(
                "traverse 回退目标修正: 使用下一条轨道x作为回退终点，"
                f"target_x={float(target_x_m):+.3f} m({target_source}), "
                f"current_row_start_x={float(original_start_fast_position[0]):+.3f} m, "
                f"delta_x={delta_x_mm:+.1f} mm, "
                f"row_y保持={float(original_start_fast_position[1]):+.3f} m"
            )
            self.start_fast_position = target_start_position
            self.active_return_target_x_m = float(target_x_m)
            self.active_return_target_source = target_source
            try:
                return super().run_reverse_phase()
            finally:
                self.start_fast_position = original_start_fast_position
                self.active_return_target_x_m = previous_target_x_m
                self.active_return_target_source = previous_target_source

        if target_source == "rail_reference_next_last_or_missing":
            rospy.loginfo("traverse 回退目标修正: 当前没有下一条轨道x，沿用原始 Flag_4_2 回零目标")

        if (
            not self.use_first_aligned_x_return
            or self.first_aligned_return_x_m is None
            or self.start_fast_position is None
        ):
            return super().run_reverse_phase()

        target_x_m = float(self.first_aligned_return_x_m)
        target_start_position = (
            target_x_m,
            float(original_start_fast_position[1]),
            float(original_start_fast_position[2]),
        )
        delta_x_mm = (float(original_start_fast_position[0]) - target_x_m) * 1000.0
        rospy.loginfo(
            "traverse 回退目标修正: 使用首次对轨x作为回退终点，"
            f"target_x={target_x_m:+.3f} m, "
            f"current_row_start_x={float(original_start_fast_position[0]):+.3f} m, "
            f"delta_x={delta_x_mm:+.1f} mm, "
            f"row_y保持={float(original_start_fast_position[1]):+.3f} m"
        )

        self.start_fast_position = target_start_position
        try:
            return super().run_reverse_phase()
        finally:
            self.start_fast_position = original_start_fast_position

    def _capture_navigation_success_heading_reference(self, reason):
        if self.navigation_heading_locked and self.heading_reference_deg is not None:
            rospy.loginfo(
                f"{reason}: 导航成功基准角已锁定 yaw={float(self.heading_reference_deg):+.2f} deg，"
                "后续轮次不再重复记录"
            )
            return True

        heading_snapshot = self.get_heading_yaw_deg()
        if heading_snapshot is None:
            rospy.logwarn(f"{reason}: 无法读取当前 {self.heading_topic}，不能记录导航成功基准角")
            return False

        current_angle_deg = float(heading_snapshot["yaw_deg"])
        position = heading_snapshot.get("position")
        self.heading_reference_deg = current_angle_deg
        self.heading_reference_source = "nav_success_fast"
        self.heading_reference_capture_wall_time = time.time()
        self.current_cycle_pre_uprail_angle_deg = current_angle_deg
        self.navigation_heading_locked = True
        self.navigation_success_position = tuple(position) if position is not None else None
        rospy.loginfo(
            f"{reason}: 已锁定导航成功基准角 yaw={current_angle_deg:+.2f} deg, "
            f"source={self.heading_topic}, pos={self._format_position_text(position)}, "
            f"age={heading_snapshot['age_sec']:.3f}s，后续所有轨道都沿用这个 yaw"
        )
        return True

    def _prepare_alignment_after_nav_success(self, reason):
        if self.nav_arrive_settle_sec > 0:
            rospy.sleep(self.nav_arrive_settle_sec)

        if not self._capture_navigation_success_heading_reference(reason):
            rospy.logerr(f"{reason}: 记录导航成功基准角失败，不开放对轨")
            with self.nav_lock:
                self.nav_failed = True
            self.publish_alignment_enable(False, f"{reason}: 记录导航成功基准角失败")
            return False

        if self.restore_heading_before_alignment:
            rospy.loginfo(f"{reason}: 先恢复到导航成功基准角，再开始对轨")
            if not self.execute_post_return_angle_restore():
                rospy.logerr(f"{reason}: 恢复导航成功基准角失败，不开放对轨")
                with self.nav_lock:
                    self.nav_failed = True
                self.publish_alignment_enable(False, f"{reason}: 回导航成功基准角失败")
                return False
        else:
            rospy.loginfo(f"{reason}: 已禁用导航后回导航成功基准角，直接进入对轨")

        if not self.switch_mode_if_needed(self.MODE_LATERAL):
            rospy.logerr(f"{reason}: 切换到等待/横移模式失败，不开放对轨")
            with self.nav_lock:
                self.nav_failed = True
            self.publish_alignment_enable(False, f"{reason}: 切换等待模式失败")
            return False

        self.publish_alignment_enable(True, reason)
        return True

    def nav_status_callback(self, msg):
        status_value = int(msg.status)
        status_name = {
            roverGoalStatus.PENDING: "PENDING",
            roverGoalStatus.ACTIVE: "ACTIVE",
            roverGoalStatus.SUCCEEDED: "SUCCEEDED",
            roverGoalStatus.ABORTED: "ABORTED",
            roverGoalStatus.REJECTED: "REJECTED",
            roverGoalStatus.LOST: "LOST",
        }.get(status_value, f"STATUS_{status_value}")

        if self.wait_nav_ready_only:
            with self.nav_lock:
                traversal_finished = self.nav_finished or self.nav_failed
                already_waiting_alignment = self.nav_waiting_for_alignment
                success_consumed = self.nav_success_consumed
                self.nav_last_status = status_value
                self.nav_last_text = status_name

            if traversal_finished:
                return
            if status_value != self.nav_success_value:
                return
            if success_consumed:
                rospy.loginfo_throttle(
                    2.0,
                    f"忽略重复导航完成状态: topic={self.nav_status_topic}, "
                    f"status={status_name}({status_value})，已进入遍历流程"
                )
                return
            if already_waiting_alignment:
                return

            with self.nav_lock:
                self.nav_waiting_for_alignment = True
                self.nav_ready_received = True
                self.nav_success_consumed = True

            rospy.loginfo(
                f"收到导航完成状态: topic={self.nav_status_topic}, "
                f"status={status_name}({status_value})，准备先回导航成功基准角再对轨"
            )
            self._prepare_alignment_after_nav_success("收到导航完成状态")
            return

        with self.nav_lock:
            goal = self.nav_current_goal
            already_waiting_alignment = self.nav_waiting_for_alignment
            traversal_finished = self.nav_finished or self.nav_failed

        if goal is None or traversal_finished:
            return
        if not self._status_matches_goal(msg, goal):
            return

        with self.nav_lock:
            self.nav_last_status = status_value
            self.nav_last_text = status_name
            if status_value == roverGoalStatus.ACTIVE:
                self.nav_seen_active = True

        if status_value == roverGoalStatus.ACTIVE:
            rospy.loginfo_throttle(
                1.0,
                f"导航点 {goal['label']} 进行中: status={status_name}, "
                f"x={msg.x:.3f}, y={msg.y:.3f}"
            )
            return

        if status_value == roverGoalStatus.SUCCEEDED:
            if already_waiting_alignment:
                return
            with self.nav_lock:
                self.nav_waiting_for_alignment = True
            rospy.loginfo(
                f"导航点 {goal['label']} 已到达: status={status_name}, "
                f"x={msg.x:.3f}, y={msg.y:.3f}, goal_id={int(msg.goal_id)}"
            )
            self._prepare_alignment_after_nav_success(f"{goal['label']} 已到达，开放对轨")
            return

        if status_value in (
            roverGoalStatus.ABORTED,
            roverGoalStatus.REJECTED,
            roverGoalStatus.LOST,
        ):
            self._handle_navigation_failure(status_value, status_name)

    def flag_callback(self, msg):
        if int(msg.data) != self.trigger_value:
            return

        with self.nav_lock:
            allow_trigger = self.nav_waiting_for_alignment and not self.nav_failed and not self.nav_finished
            goal = self.nav_current_goal
        if not allow_trigger:
            goal_label = goal["label"] if goal else "N/A"
            rospy.loginfo_throttle(
                1.0,
                f"忽略上轨触发: 当前未到允许对轨/上轨阶段, goal={goal_label}"
            )
            return

        super().flag_callback(msg)

    def _check_navigation_timeout(self):
        if self.nav_goal_timeout_sec <= 0:
            return

        with self.nav_lock:
            goal = self.nav_current_goal
            sent_time = self.nav_goal_sent_wall_time
            waiting_alignment = self.nav_waiting_for_alignment
            failed = self.nav_failed
        if goal is None or waiting_alignment or failed or sent_time <= 0:
            return

        elapsed = time.time() - sent_time
        if elapsed < self.nav_goal_timeout_sec:
            return

        rospy.logwarn(
            f"导航点 {goal['label']} 超时 {elapsed:.1f}s，准备按失败重试逻辑处理"
        )
        self._handle_navigation_failure(self.nav_last_status, self.nav_last_text or "TIMEOUT")

    def run(self):
        if self.wait_nav_ready_only:
            rate = rospy.Rate(max(1.0, self.loop_rate_hz))

            rospy.loginfo(f"traverse 启动后保持当前底盘模式，等待 {self.nav_status_topic} 到达成功态")

            while not rospy.is_shutdown() and self.running:
                if self.nav_failed:
                    rospy.logerr("traverse 检测到失败状态，停止后续遍历")
                    break

                should_run = False
                with self.trigger_lock:
                    if self.trigger_requested and not self.sequence_busy:
                        should_run = True

                if should_run:
                    with self.nav_lock:
                        self.nav_waiting_for_alignment = False
                    rospy.loginfo("对轨完成，开始执行上/下轨流程")
                    self.publish_alignment_enable(False, "开始执行上/下轨")
                    success = self.execute_sequence()
                    if not success:
                        with self.nav_lock:
                            self.nav_failed = True
                        rospy.logerr("上/下轨流程失败，停止遍历")
                        break
                    completed_sequence_id = int(self.sequence_counter)
                    if self._has_more_cycles_after(completed_sequence_id):
                        next_cycle_id = completed_sequence_id + 1
                        rospy.loginfo(
                            f"第 {completed_sequence_id} 轮完成，准备开始第 {next_cycle_id} 轮对轨"
                        )
                        self._open_alignment_for_next_cycle(
                            f"第{next_cycle_id}轮准备开始，对轨重新放开"
                        )
                        rate.sleep()
                        continue
                    rospy.loginfo(
                        f"已完成全部 {completed_sequence_id} 轮遍历，流程结束"
                    )
                    break

                rate.sleep()

            self.publish_alignment_enable(False, "traverse 退出")
            return

        if not self.navigation_goals:
            rospy.logwarn("未配置 navigation_goals，traverse 将退回 Flag_4_2 单点触发模式")
            self.publish_alignment_enable(True, "未配置导航点，直接允许对轨")
            return super().run()

        rate = rospy.Rate(max(1.0, self.loop_rate_hz))

        rospy.loginfo("traverse 启动后先切到等待模式")
        self.switch_mode_if_needed(self.MODE_LATERAL)

        if not self._advance_to_next_goal("startup"):
            rospy.logerr("未能启动首个导航点，traverse 退出")
            return

        while not rospy.is_shutdown() and self.running:
            if self.nav_failed:
                rospy.logerr("traverse 检测到导航失败，停止后续遍历")
                break

            self._check_navigation_timeout()

            should_run = False
            with self.trigger_lock:
                if self.trigger_requested and not self.sequence_busy:
                    should_run = True

            if should_run:
                with self.nav_lock:
                    goal = self.nav_current_goal
                    self.nav_waiting_for_alignment = False
                goal_label = goal["label"] if goal else "N/A"
                rospy.loginfo(f"导航点 {goal_label} 对轨完成，开始执行上/下轨流程")
                self.publish_alignment_enable(False, f"{goal_label} 开始执行上/下轨")
                success = self.execute_sequence()
                if not success:
                    with self.nav_lock:
                        self.nav_failed = True
                    rospy.logerr(f"导航点 {goal_label} 的上/下轨流程失败，停止遍历")
                    break
                completed_sequence_id = int(self.sequence_counter)
                if self.navigate_first_goal_only:
                    if self._has_more_cycles_after(completed_sequence_id):
                        next_cycle_id = completed_sequence_id + 1
                        rospy.loginfo(
                            f"首轨导航模式: 第 {completed_sequence_id} 轮完成，准备开始第 {next_cycle_id} 轮对轨"
                        )
                        self._open_alignment_for_next_cycle(
                            f"第{next_cycle_id}轮准备开始，对轨重新放开"
                        )
                        rate.sleep()
                        continue
                    rospy.loginfo(
                        f"首轨导航模式: 已完成全部 {completed_sequence_id} 轮遍历，流程结束"
                    )
                    break
                if not self._advance_to_next_goal(f"sequence_done_{goal_label}"):
                    break

            rate.sleep()

        self.publish_alignment_enable(False, "traverse 退出")


def main():
    auto_load_workflow_yaml()
    controller = TraverseController()
    controller.run()


if __name__ == "__main__":
    main()
