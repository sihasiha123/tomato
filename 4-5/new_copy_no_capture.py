#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""new_copy 的无采图版本。

保留原有检测、对轨、CAN 和工作流逻辑，只关闭原图抓拍落盘。
"""

import os
import threading
import time

import cv2

import rospy
import rosgraph
import rosparam
import torch
from std_msgs.msg import Int32

from new_copy import FusionController as BaseFusionController, SystemState


def wait_for_ros_master(timeout_sec=60.0):
    deadline = time.time() + max(0.0, float(timeout_sec))
    master = rosgraph.Master("/new_copy_no_capture_yaml_loader")
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
            f"[new_copy_no_capture] warning: ROS master not available within "
            f"{wait_timeout_sec:.1f}s; workflow yaml not loaded: {workflow_yaml}"
        )
        return
    try:
        param_list = rosparam.load_file(workflow_yaml)
        for params, namespace in param_list:
            rosparam.upload_params(namespace, params)
        print(f"[new_copy_no_capture] auto-loaded workflow yaml: {workflow_yaml}")
    except Exception as exc:
        print(
            f"[new_copy_no_capture] warning: failed to auto-load workflow yaml: "
            f"{workflow_yaml}, error={exc}"
        )


class FusionControllerNoCapture(BaseFusionController):
    def __init__(self, *args, **kwargs):
        os.environ["RAIL_RAW_CAPTURE_ENABLE"] = "0"
        super().__init__(*args, **kwargs)
        self.raw_capture_enabled = False
        self.raw_capture_session_dir = None
        self.raw_capture_count = 0
        self.last_raw_capture_time = 0.0
        self.alignment_gate_enabled = bool(
            rospy.get_param(
                "~use_alignment_gate",
                rospy.get_param(f"{self.workflow_param_ns}/use_alignment_gate", False),
            )
        )
        self.alignment_enable_topic = rospy.get_param(
            "~alignment_enable_topic",
            rospy.get_param(
                f"{self.workflow_param_ns}/alignment_enable_topic",
                "/traverse/alignment_enabled",
            ),
        )
        self.alignment_enabled = bool(
            rospy.get_param(
                "~alignment_start_enabled",
                rospy.get_param(
                    f"{self.workflow_param_ns}/alignment_start_enabled",
                    not self.alignment_gate_enabled,
                ),
            )
        )
        self.alignment_enable_sub = None
        if self.alignment_gate_enabled:
            self.alignment_enable_sub = rospy.Subscriber(
                self.alignment_enable_topic,
                Int32,
                self.alignment_enable_callback,
                queue_size=8,
            )
            # traverse 负责整轮节奏控制，这里保持无限轮等待即可。
            self.workflow_cycle_count = 0
            if not self.alignment_enabled:
                self.workflow_status_text = "WAIT_NAV"
                self.last_stage_name = "WAIT_NAV"
                self.last_motion_text = "WAIT_NAV"
            rospy.loginfo(
                f"🧭 已启用 traverse 对轨门控: topic={self.alignment_enable_topic}, "
                f"start_enabled={self.alignment_enabled}"
            )
        self.wait_nav_infer_enabled = (
            os.environ.get("RAIL_WAIT_NAV_INFER_ENABLE", "0").strip().lower()
            in {"1", "true", "yes", "on"}
        )
        default_yolo_max_fps = "8.0" if self.alignment_gate_enabled else "0.0"
        self.yolo_max_fps = max(
            0.0,
            float(os.environ.get("RAIL_YOLO_MAX_FPS", default_yolo_max_fps)),
        )
        self.yolo_min_interval_sec = (
            0.0 if self.yolo_max_fps <= 0.0 else 1.0 / self.yolo_max_fps
        )
        self.last_infer_wall_time = 0.0
        self.last_infer_duration_ms = 0.0
        wait_nav_display_hz = float(os.environ.get("RAIL_WAIT_NAV_DISPLAY_HZ", "2.0"))
        self.wait_nav_display_interval_sec = 1.0 / max(0.1, wait_nav_display_hz)
        self.last_wait_nav_display_time = 0.0
        rospy.loginfo(
            f"🤖 YOLO 运行策略: wait_nav_infer={'on' if self.wait_nav_infer_enabled else 'off'}, "
            f"max_fps={self.yolo_max_fps:.1f}"
        )
        rospy.loginfo("📷 已切换到无采图版本: 不保存任何原图")

    def maybe_save_raw_frame(self, frame):
        return

    def _reset_alignment_cycle_runtime(self, reason):
        self.stop_motion()
        self.frame_buffer.clear()
        self.stable_lateral_error_mm = 0
        self.last_pixel_offset_px = 0.0
        self.last_rail_center_x = self.img_width / 2.0
        self.last_target_center_x = self.pixel_lateral_calculator.target_center_x
        self.last_rail_width_px = 0.0
        self.last_step_mm = 0.0
        self.last_accepted_pixel_offset_px = None
        self.last_accepted_lateral_error_mm = None
        self.force_realign_move_pending = False
        self.fitted_lines_pixel = []
        self.yolo_expected_left_x = None
        self.yolo_expected_right_x = None
        self.yolo_prev_left_line = None
        self.yolo_prev_right_line = None
        self.yolo_prev_gauge_px = None
        self.last_display_fit_lines = []
        self.last_display_center_line = None
        self.last_extract_reason = reason
        with self.state_lock:
            self.current_state = SystemState.IDLE

    def _show_gate_idle_frame(self, frame, text, paused):
        now = time.time()
        if now - self.last_wait_nav_display_time >= self.wait_nav_display_interval_sec:
            self.last_wait_nav_display_time = now
            vis_frame = frame.copy()
            cv2.putText(
                vis_frame,
                text,
                (self.img_width // 2 - 150, self.img_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (0, 180, 255),
                4,
            )
            if paused:
                cv2.putText(
                    vis_frame,
                    "PAUSED",
                    (self.img_width // 2 - 100, self.img_height // 2 + 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2,
                    (0, 0, 255),
                    4,
                )
            cv2.namedWindow("Rail Alignment new_copy", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Rail Alignment new_copy", 1600, 900)
            cv2.imshow("Rail Alignment new_copy", vis_frame)

    def _handle_runtime_keys(self, paused):
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rospy.loginfo("   用户退出")
            return paused, True
        if key == ord("r"):
            self._reset_alignment_cycle_runtime("RESET")
            if self.alignment_enabled:
                self.workflow_status_text = "ALIGNING"
                self.last_stage_name = "COLLECT"
                self.last_motion_text = "WAITING"
            else:
                self.workflow_status_text = "WAIT_NAV"
                self.last_stage_name = "WAIT_NAV"
                self.last_motion_text = "WAIT_NAV"
            rospy.loginfo("   系统已重置")
            return paused, False
        if key == ord("p"):
            paused = not paused
            if paused:
                self.stop_motion()
                rospy.loginfo("⏸️  系统暂停")
            else:
                rospy.loginfo("▶️  系统继续")
        return paused, False

    def alignment_enable_callback(self, msg):
        enabled = int(msg.data) != 0
        if enabled == self.alignment_enabled:
            return

        self.alignment_enabled = enabled
        with self.workflow_lock:
            workflow_busy = self.workflow_busy

        if enabled:
            rospy.loginfo("🔓 traverse 已放开对轨，对轨流程开始工作")
            if not workflow_busy:
                self.workflow_finished = False
                self.workflow_failed = False
                self.workflow_status_text = "ALIGNING"
                self.last_stage_name = "COLLECT"
                self.last_motion_text = "WAITING"
                self._reset_alignment_cycle_runtime("TRAVERSE_ENABLE")
        else:
            rospy.loginfo("🔒 traverse 已关闭对轨，等待导航到位")
            if not workflow_busy:
                self.workflow_status_text = "WAIT_NAV"
                self.last_stage_name = "WAIT_NAV"
                self.last_motion_text = "WAIT_NAV"
                self._reset_alignment_cycle_runtime("TRAVERSE_WAIT_NAV")

    def run(self):
        if not self.alignment_gate_enabled:
            return super().run()

        self.camera.start()
        rospy.loginfo("⏳ 等待相机启动...")
        time.sleep(2)

        with self.state_lock:
            self.current_state = SystemState.IDLE

        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

        paused = False

        try:
            while not rospy.is_shutdown():
                frame = self.camera.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                self.stats["total_frames"] += 1
                with self.workflow_lock:
                    workflow_busy = self.workflow_busy
                with self.state_lock:
                    current_state = self.current_state

                if current_state == SystemState.ALIGNED and not workflow_busy:
                    rospy.loginfo_throttle(2.0, "✅ 对轨已成功，立即暂停轨道处理并触发工作流")
                    self.maybe_start_workflow_cycle()
                    with self.workflow_lock:
                        workflow_busy = self.workflow_busy

                if not self.alignment_enabled and not self.wait_nav_infer_enabled:
                    if not workflow_busy:
                        self.workflow_status_text = "WAIT_NAV"
                        self.last_stage_name = "WAIT_NAV"
                        self.last_motion_text = "WAIT_NAV"
                        with self.state_lock:
                            self.current_state = SystemState.IDLE
                    self._show_gate_idle_frame(frame, "WORKFLOW" if workflow_busy else "WAIT NAV", paused)
                    paused, should_break = self._handle_runtime_keys(paused)
                    if should_break:
                        break
                    time.sleep(0.02)
                    continue

                if workflow_busy or current_state == SystemState.ALIGNED:
                    self._show_gate_idle_frame(frame, "WORKFLOW" if workflow_busy else "ALIGNED", paused)
                    paused, should_break = self._handle_runtime_keys(paused)
                    if should_break:
                        break
                    time.sleep(0.02)
                    continue

                model_frame = self.preprocess_frame_for_model(frame)
                now = time.time()
                if (
                    self.yolo_min_interval_sec > 0.0 and
                    self.last_infer_wall_time > 0.0 and
                    (now - self.last_infer_wall_time) < self.yolo_min_interval_sec
                ):
                    rospy.loginfo_throttle(
                        5.0,
                        f"⏱️ YOLO限频生效: max_fps={self.yolo_max_fps:.1f}, "
                        f"last_infer={self.last_infer_duration_ms:.1f}ms"
                    )
                    vis_frame = self.create_visualization(frame, None)
                    if not self.alignment_enabled:
                        cv2.putText(
                            vis_frame,
                            "WAIT NAV",
                            (self.img_width // 2 - 150, self.img_height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            2,
                            (0, 180, 255),
                            4,
                        )
                    if paused:
                        cv2.putText(
                            vis_frame,
                            "PAUSED",
                            (self.img_width // 2 - 100, self.img_height // 2 + 80),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            2,
                            (0, 0, 255),
                            4,
                        )
                    cv2.namedWindow("Rail Alignment new_copy", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("Rail Alignment new_copy", 1600, 900)
                    cv2.imshow("Rail Alignment new_copy", vis_frame)
                    paused, should_break = self._handle_runtime_keys(paused)
                    if should_break:
                        break
                    time.sleep(0.005)
                    continue

                infer_start = time.time()
                detection_result = self.model.predict(
                    model_frame,
                    conf=self.yolo_conf,
                    imgsz=self.yolo_imgsz,
                    device=self.yolo_device,
                    verbose=False,
                    save=False,
                )[0]
                self.last_infer_wall_time = time.time()
                self.last_infer_duration_ms = (self.last_infer_wall_time - infer_start) * 1000.0
                rospy.loginfo_throttle(
                    5.0,
                    f"🤖 YOLO推理耗时: {self.last_infer_duration_ms:.1f}ms, "
                    f"device={self.yolo_device}, imgsz={self.yolo_imgsz}, conf={self.yolo_conf:.2f}"
                )

                if current_state != SystemState.COLLECTING:
                    self.update_live_display_cache(detection_result)

                if not paused and self.alignment_enabled:
                    if current_state == SystemState.IDLE:
                        self.frame_buffer.clear()
                        self.last_stage_name = "COLLECT"
                        with self.state_lock:
                            self.current_state = SystemState.COLLECTING

                    elif current_state == SystemState.COLLECTING:
                        measure = self.extract_pixel_measurement(detection_result)
                        if measure is not None:
                            self.apply_measurement(measure)
                            self.frame_buffer.append(measure)
                            inferred_text = " 推中线" if measure.get("inferred_from_single", False) else ""
                            rospy.loginfo(
                                f"   像素采样 {len(self.frame_buffer)}/{self.max_buffer_size}"
                                f"  中线={measure['rail_center_x']:.1f}px"
                                f"  目标={measure['target_center_x']:.1f}px"
                                f"  偏差={measure['pixel_offset_px']:+.1f}px"
                                f"  轨宽={measure['rail_width_px']:.1f}px"
                                f"{inferred_text}"
                            )

                        if len(self.frame_buffer) >= self.max_buffer_size:
                            rospy.loginfo("   像素采样完成")
                            stable_measure = self.filter_collected_data()

                            if stable_measure is not None:
                                self.apply_measurement(stable_measure)
                                self.stats["successful_detections"] += 1
                                with self.state_lock:
                                    self.current_state = SystemState.PROCESSING
                            else:
                                rospy.logwarn("⚠️ 滤波失败,重新收集")
                                with self.state_lock:
                                    self.current_state = SystemState.IDLE
                elif not self.alignment_enabled and not self.workflow_busy:
                    self.workflow_status_text = "WAIT_NAV"
                    self.last_stage_name = "WAIT_NAV"
                    self.last_motion_text = "WAIT_NAV"
                    with self.state_lock:
                        self.current_state = SystemState.IDLE

                vis_frame = self.create_visualization(frame, detection_result)

                if not self.alignment_enabled:
                    cv2.putText(
                        vis_frame,
                        "WAIT NAV",
                        (self.img_width // 2 - 150, self.img_height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        2,
                        (0, 180, 255),
                        4,
                    )
                if paused:
                    cv2.putText(
                        vis_frame,
                        "PAUSED",
                        (self.img_width // 2 - 100, self.img_height // 2 + 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        2,
                        (0, 0, 255),
                        4,
                    )

                cv2.namedWindow("Rail Alignment new_copy", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Rail Alignment new_copy", 1600, 900)
                cv2.imshow("Rail Alignment new_copy", vis_frame)

                paused, should_break = self._handle_runtime_keys(paused)
                if should_break:
                    break

        except KeyboardInterrupt:
            rospy.loginfo("\n⛔ 用户中断")
        except Exception as e:
            rospy.logerr(f"❌ 运行错误: {e}")
            import traceback

            traceback.print_exc()
        finally:
            self.running = False
            self.stop_motion()
            if self.control_thread:
                self.control_thread.join(timeout=2)
            if self.workflow_thread:
                self.workflow_thread.join(timeout=2)
            self.camera.release()
            cv2.destroyAllWindows()
            if self.can_bus:
                self.can_bus.shutdown()
            rospy.loginfo("\n   程序结束")


def main():
    auto_load_workflow_yaml()
    yolo_weights = os.environ.get(
        "RAIL_YOLO_WEIGHTS",
        "/home/ubuntu/mmLaneDet-master/work_dirs/yolov8s_421_from_new_e80/weights/best.pt",
    )
    requested_device = os.environ.get("RAIL_YOLO_DEVICE", "auto").strip().lower()
    if requested_device in {"", "auto"}:
        device = "0" if torch.cuda.is_available() else "cpu"
    elif requested_device != "cpu" and not torch.cuda.is_available():
        rospy.logwarn(f"请求设备 {requested_device} 但当前无 CUDA, 自动切换到 CPU")
        device = "cpu"
    else:
        device = requested_device

    yolo_imgsz = int(os.environ.get("RAIL_YOLO_IMGSZ", "960"))
    yolo_conf = float(os.environ.get("RAIL_YOLO_CONF", "0.20"))
    yolo_min_y = int(os.environ.get("RAIL_YOLO_MIN_Y", "530"))
    yolo_row_step = int(os.environ.get("RAIL_YOLO_ROW_STEP", "10"))

    try:
        controller = FusionControllerNoCapture(
            yolo_weights=yolo_weights,
            device=device,
            yolo_imgsz=yolo_imgsz,
            yolo_conf=yolo_conf,
            yolo_min_y=yolo_min_y,
            yolo_row_step=yolo_row_step,
        )
        controller.run()
    except Exception as e:
        rospy.logerr(f"❌ 系统启动失败: {e}")
        import traceback

        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    main()
