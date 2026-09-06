import sys
import os
import cv2
import time
import rospy
import can
import numpy as np
import torch
from collections import deque
from enum import Enum
import threading
from std_msgs.msg import Int32

# ========== 配置参数 ==========
BASE_PATH = '/home/ubuntu/OrbbecSDK_Python_v1.1.4_linux_x64_release/python3.8'
sys.path.insert(0, f'{BASE_PATH}/Samples')
os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH', '') + f':{BASE_PATH}/lib/c_lib'

from ObTypes import *
from Property import *
import Context
import Pipeline
import StreamProfile

def configure_orbbec_sdk_logging():
    """Suppress noisy Orbbec SDK warnings while keeping real errors visible."""
    try:
        ctx = Context.Context(None)
        ctx.setLoggerSeverity(OB_PY_LOG_SEVERITY_ERROR)
    except Exception as exc:
        print(f"[new_copy] warning: failed to configure Orbbec SDK log level: {exc}")

configure_orbbec_sdk_logging()

sys.path.append('/home/ubuntu/mmLaneDet-master')
os.environ.setdefault('YOLO_CONFIG_DIR', '/home/ubuntu/mmLaneDet-master/.yolo_cfg')
from ultralytics import YOLO
from demo.yolo_rail_extract import (
    choose_best_mask,
    sample_line_points,
    fit_line_from_points,
    line_to_points,
    ema,
)

# ========== 状态枚举 ==========
class SystemState(Enum):
    IDLE = 0
    COLLECTING = 1
    PROCESSING = 2
    ALIGNED = 5
    ERROR = 6

# ========== 单位转换工具类 ==========
class UnitConverter:
    """hall和mm之间的转换"""
    HALL_TO_MM = 2.38

    @staticmethod
    def mm_to_hall(distance_mm):
        return int(round(distance_mm / UnitConverter.HALL_TO_MM))

    @staticmethod
    def hall_to_mm(distance_hall):
        return float(distance_hall) * UnitConverter.HALL_TO_MM

# ========== 相机内参类 ==========
class CameraCalibrationFor1080p:
    def __init__(self):
        self.fx = 1358.4
        self.fy = 1358.6
        self.cx = 982.9874
        self.cy = 536.1308
        self.k1 = 0.0974
        self.k2 = -0.1753
        self.p1 = 0.0
        self.p2 = 0.0
        self.camera_height = 305
        self.camera_lateral_offset = -10
        self.pitch =10.1
        self.roll = 0.0
        self.yaw = 0.0

        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            [self.k1, self.k2, self.p1, self.p2]
        ], dtype=np.float32)

        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ 相机标定参数加载完成 (正装模式)")
        rospy.loginfo(f"   高度={self.camera_height}mm, 横向偏移={self.camera_lateral_offset}mm")
        rospy.loginfo("=" * 60)

# ========== 相机类 ==========
class OrbbecCamera:
    def __init__(self, camera_calib, enable_undistort=True):
        self.pipe = None
        self.config = None
        self.enable_undistort = enable_undistort
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.capture_thread = None
        self.is_running = False

        if self.enable_undistort:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                camera_calib.camera_matrix,
                camera_calib.dist_coeffs,
                None,
                camera_calib.camera_matrix,
                (1920, 1080),
                cv2.CV_16SC2
            )
        self._init_camera()

    def _init_camera(self):
        try:
            self.pipe = Pipeline.Pipeline(None, None)
            self.config = Pipeline.Config()
            profiles = self.pipe.getStreamProfileList(OB_PY_SENSOR_COLOR)
            for i in range(profiles.count()):
                try:
                    profile = profiles.getProfile(i).toConcreteStreamProfile(OB_PY_STREAM_VIDEO)
                    if profile.width() == 1920 and profile.height() == 1080:
                        self.config.enableStream(profile)
                        return
                except:
                    continue
            raise Exception("无法启用1080p配置")
        except Exception as e:
            rospy.logerr(f"相机初始化失败: {e}")
            raise

    def _capture_loop(self):
        self.pipe.start(self.config, None)
        while self.is_running:
            frameSet = self.pipe.waitForFrames(200)
            if frameSet is None:
                continue
            colorFrame = frameSet.colorFrame()
            if colorFrame is not None:
                data = colorFrame.data()
                frame_data = np.frombuffer(data, dtype=np.uint8)

                if colorFrame.format() == OB_PY_FORMAT_MJPG:
                    frame = cv2.imdecode(frame_data, 1)
                elif colorFrame.format() == OB_PY_FORMAT_RGB888:
                    frame = frame_data.reshape((1080, 1920, 3))
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif colorFrame.format() == OB_PY_FORMAT_YUYV:
                    frame = frame_data.reshape((1080, 1920, 2))
                    frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
                else:
                    continue

                if self.enable_undistort:
                    frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

                with self.frame_lock:
                    self.latest_frame = frame
        self.pipe.stop()

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def stop(self):
        self.is_running = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2)

    def get_frame(self):
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None

    def release(self):
        self.stop()

# ========== 像素横移计算器类 ==========
class PixelLateralCalculator:
    """
    基于像素坐标系计算横移偏差
    """
    def __init__(self, img_width=1920, img_height=1080, target_center_offset_px=0.0):
        self.img_width = img_width
        self.img_height = img_height
        self.screen_center_x = img_width / 2.0
        self.target_center_offset_px = float(target_center_offset_px)
        self.target_center_x = self.screen_center_x + self.target_center_offset_px

        # 像素到mm的转换系数 (需要根据实际标定调整)
        self.PIXEL_TO_MM_RATIO = 0.25

        # 采样行: 在图像下半部分取样
        self.sample_row = int(img_height * 0.75)  # y=810

        rospy.loginfo("=" * 60)
        rospy.loginfo("✅ 像素横移计算器初始化")
        rospy.loginfo(f"   屏幕中心: x={self.screen_center_x:.1f}px")
        rospy.loginfo(f"   目标中心: x={self.target_center_x:.1f}px (offset={self.target_center_offset_px:+.1f}px)")
        rospy.loginfo(f"   采样行: y={self.sample_row}px")
        rospy.loginfo(f"   像素转mm系数: {self.PIXEL_TO_MM_RATIO} mm/px")
        rospy.loginfo("=" * 60)

    def measure_from_pixel(
        self,
        fitted_lines_pixel,
        detection_mode,
        lane_roles=None,
        fallback_gauge_px=None,
        default_gauge_px=None,
    ):
        """仅基于像素拟合线计算当前偏差，不使用 IPM/角度。"""
        if fitted_lines_pixel is None or len(fitted_lines_pixel) == 0:
            return None

        y = self.sample_row
        rail_width_px = None
        inferred_from_single = False

        if detection_mode == "Dual-Rail" and len(fitted_lines_pixel) >= 2:
            k1, b1 = fitted_lines_pixel[0]
            k2, b2 = fitted_lines_pixel[1]
            x1 = k1 * y + b1
            x2 = k2 * y + b2
            left_x = min(x1, x2)
            right_x = max(x1, x2)
            rail_center_x = (left_x + right_x) / 2.0
            rail_width_px = right_x - left_x
        elif len(fitted_lines_pixel) >= 1:
            k, b = fitted_lines_pixel[0]
            line_x = k * y + b
            role = lane_roles[0] if lane_roles and len(lane_roles) >= 1 else None
            if fallback_gauge_px is not None:
                gauge_px = float(fallback_gauge_px)
            elif default_gauge_px is not None:
                gauge_px = float(default_gauge_px)
            else:
                gauge_px = None
            if role in ("left", "right") and gauge_px is not None and 80.0 <= gauge_px <= 1100.0:
                if role == "left":
                    rail_center_x = line_x + gauge_px / 2.0
                else:
                    rail_center_x = line_x - gauge_px / 2.0
                rail_width_px = gauge_px
                inferred_from_single = True
            else:
                return None
        else:
            return None

        pixel_offset = rail_center_x - self.target_center_x
        lateral_mm = pixel_offset * self.PIXEL_TO_MM_RATIO
        direction = 1 if pixel_offset >= 0 else -1

        return {
            "rail_center_x": float(rail_center_x),
            "target_center_x": float(self.target_center_x),
            "rail_width_px": float(rail_width_px),
            "pixel_offset_px": float(pixel_offset),
            "abs_pixel_offset_px": abs(float(pixel_offset)),
            "lateral_mm": float(lateral_mm),
            "direction": int(direction),
            "inferred_from_single": bool(inferred_from_single),
        }


# ========== 主控制器类 ==========
class FusionController:
    def __init__(
        self,
        yolo_weights,
        device='0',
        yolo_imgsz=960,
        yolo_conf=0.20,
        yolo_min_y=530,
        yolo_row_step=10,
    ):
        rospy.init_node('rail_alignment_v7_7_new_copy', anonymous=True)

        rospy.loginfo("=" * 70)
        rospy.loginfo(" 轨道对齐系统 V7.7 new_copy - 控制改版副本")
        rospy.loginfo(" ✅ 模式: 双轨拟合中线 + 像素横移")
        rospy.loginfo(" ✅ 已删除: 角度补偿 / IPM 误差控制 / 重采集补偿流程")
        rospy.loginfo(" ✅ 控制: 全程只用 0x0E 位置模式")
        rospy.loginfo(" ✅ 步长: 每次固定 2cm, 每步后重新拟合再决定方向")
        rospy.loginfo("=" * 70)

        self.model = YOLO(yolo_weights)
        rospy.loginfo(f"   使用YOLO权重: {yolo_weights}")
        self.yolo_device = device
        self.yolo_imgsz = int(yolo_imgsz)
        self.yolo_conf = float(yolo_conf)
        self.yolo_min_y = int(yolo_min_y)
        self.yolo_row_step = int(yolo_row_step)
        self.yolo_side_prior = True
        self.yolo_side_margin = 0.3
        self.yolo_min_points = 6
        self.yolo_fit_residual = 20.0
        self.yolo_ema_alpha = 0.6
        self.yolo_expected_left_x = None
        self.yolo_expected_right_x = None
        self.yolo_prev_left_line = None
        self.yolo_prev_right_line = None
        self.yolo_prev_gauge_px = None
        self.model_exposure_enable = os.environ.get(
            'RAIL_MODEL_EXPOSURE_ENABLE',
            '1',
        ).strip() != '0'
        self.model_exposure_clahe_clip = float(os.environ.get('RAIL_MODEL_EXPOSURE_CLAHE_CLIP', '2.0'))
        self.model_exposure_clahe_tile = max(2, int(os.environ.get('RAIL_MODEL_EXPOSURE_CLAHE_TILE', '8')))
        self.model_exposure_gamma = float(os.environ.get('RAIL_MODEL_EXPOSURE_GAMMA', '0.75'))
        self.model_exposure_alpha = float(os.environ.get('RAIL_MODEL_EXPOSURE_ALPHA', '1.0'))
        self.model_exposure_beta = float(os.environ.get('RAIL_MODEL_EXPOSURE_BETA', '0.0'))
        self.model_exposure_log_interval_sec = float(
            os.environ.get('RAIL_MODEL_EXPOSURE_LOG_INTERVAL_SEC', '5.0')
        )
        self._model_exposure_last_log_time = 0.0
        self._model_exposure_gamma_lut = self._build_gamma_lut(self.model_exposure_gamma)
        if self.model_exposure_enable:
            rospy.loginfo(
                "🌙 已启用模型前曝光增强: "
                f"clahe_clip={self.model_exposure_clahe_clip:.2f}, "
                f"tile={self.model_exposure_clahe_tile}, "
                f"gamma={self.model_exposure_gamma:.2f}, "
                f"alpha={self.model_exposure_alpha:.2f}, "
                f"beta={self.model_exposure_beta:.1f}"
            )
        self.raw_capture_enabled = os.environ.get('RAIL_RAW_CAPTURE_ENABLE', '1').strip() != '0'
        self.raw_capture_interval_sec = float(os.environ.get('RAIL_RAW_CAPTURE_INTERVAL_SEC', '2.0'))
        self.raw_capture_root_dir = os.environ.get(
            'RAIL_RAW_CAPTURE_ROOT',
            '/home/ubuntu/mmLaneDet-master/demo/rail_data/live_raw_capture',
        )
        self.raw_capture_session_dir = None
        self.raw_capture_count = 0
        self.last_raw_capture_time = 0.0
        if self.raw_capture_enabled:
            session_name = time.strftime('session_%Y%m%d_%H%M%S')
            self.raw_capture_session_dir = os.path.join(self.raw_capture_root_dir, session_name)
            os.makedirs(self.raw_capture_session_dir, exist_ok=True)
            rospy.loginfo(f"📸 已启用原图抓拍: 每{self.raw_capture_interval_sec:.1f}s 保存一张")
            rospy.loginfo(f"   保存目录: {self.raw_capture_session_dir}")
        self.camera_calib = CameraCalibrationFor1080p()
        self.camera = OrbbecCamera(self.camera_calib, enable_undistort=True)
        self.target_center_offset_px = float(os.environ.get('RAIL_TARGET_CENTER_OFFSET_PX', '0.0'))
        # 像素横移计算器
        self.pixel_lateral_calculator = PixelLateralCalculator(
            img_width=1920,
            img_height=1080,
            target_center_offset_px=self.target_center_offset_px,
        )

        # CAN总线配置
        self.can_id = 0x00A
        self.current_mode = None
        self.MODE_LATERAL = 0x04

        # 查询参数
        self.MODE_SWITCH_QUERY_INTERVAL = 0.15
        self.MOTION_QUERY_INTERVAL = 0.25  # 稍微缩短查询间隔
        self.MODE_SWITCH_WAIT_TIME = 3.0
        self.QUERY_TIMEOUT = 10.0
        self.DEBUG_MODE = True
        self.pixel_deadband_px = float(os.environ.get('RAIL_PIXEL_DEADBAND_PX', '8.0'))
        self.pixel_direction_sign = 1
        self.force_realign_after_cycle_shift = os.environ.get(
            'RAIL_FORCE_REALIGN_AFTER_CYCLE_SHIFT',
            '1',
        ).strip() != '0'
        self.force_realign_min_px = float(os.environ.get('RAIL_FORCE_REALIGN_MIN_PX', '0.5'))
        self.default_single_lane_gauge_px = float(
            os.environ.get('RAIL_DEFAULT_SINGLE_LANE_GAUGE_PX', '900.0')
        )
        self.pixel_step_speed_hall = int(os.environ.get('RAIL_PIXEL_STEP_SPEED_HALL', '6'))
        self.pixel_step_mm = float(os.environ.get('RAIL_PIXEL_STEP_MM', '20.0'))
        self.pixel_small_error_threshold_px = float(
            os.environ.get('RAIL_PIXEL_SMALL_ERROR_THRESHOLD_PX', '30.0')
        )
        self.pixel_small_error_step_mm = float(
            os.environ.get('RAIL_PIXEL_SMALL_ERROR_STEP_MM', '5.0')
        )
        self.pixel_step_settle_sec = float(os.environ.get('RAIL_PIXEL_STEP_SETTLE_SEC', '0.25'))
        rospy.loginfo(
            f"   像素横移策略: 死区={self.pixel_deadband_px:.1f}px, "
            f"小误差<{self.pixel_small_error_threshold_px:.1f}px -> {self.pixel_small_error_step_mm:.1f}mm, "
            f"其余 -> {self.pixel_step_mm:.1f}mm"
        )
        self.flag_topic = rospy.get_param("~flag_topic", "/flag1")
        self.done_topic = rospy.get_param("~done_topic", "/rail_cycle_done")
        self.workflow_param_ns = rospy.get_param("~workflow_param_ns", "/rail_workflow")
        self.workflow_enable_auto = bool(rospy.get_param("~enable_auto_workflow", True))
        self.workflow_cycle_count = int(
            rospy.get_param(
                "~workflow_cycle_count",
                rospy.get_param(f"{self.workflow_param_ns}/cycle_count", 2),
            )
        )
        self.workflow_trigger_value = int(rospy.get_param("~trigger_value", 1))
        self.workflow_flag_rate_hz = float(rospy.get_param("~flag_rate_hz", 10.0))
        self.workflow_flag_duration_sec = float(rospy.get_param("~flag_duration_sec", 3.0))
        self.workflow_flag_reset_duration_sec = float(rospy.get_param("~flag_reset_duration_sec", 0.6))
        self.workflow_done_timeout_sec = float(rospy.get_param("~done_timeout_sec", 0.0))
        self.workflow_inter_cycle_delay_sec = float(rospy.get_param("~inter_cycle_delay_sec", 0.5))
        self.workflow_wait_subscriber_sec = float(rospy.get_param("~wait_subscriber_sec", 3.0))
        self.workflow_stop_on_failure = bool(rospy.get_param("~stop_on_failure", True))

        # 像素坐标系拟合结果
        self.fitted_lines_pixel = []

        # 系统状态
        self.img_width, self.img_height = 1920, 1080
        self.max_buffer_size = 5
        self.frame_buffer = deque(maxlen=self.max_buffer_size)  # 满了自动丢最旧
        self.current_state = SystemState.IDLE
        self.state_lock = threading.Lock()

        self.stable_lateral_error_mm = 0
        self.last_pixel_offset_px = 0.0
        self.last_rail_center_x = self.img_width / 2.0
        self.last_target_center_x = self.pixel_lateral_calculator.target_center_x
        self.last_rail_width_px = 0.0
        self.last_step_mm = 0.0
        self.last_accepted_lateral_error_mm = None
        self.last_accepted_pixel_offset_px = None
        self.max_pixel_jump_px = float(os.environ.get('RAIL_MAX_PIXEL_JUMP_PX', '120.0'))
        self.smoothing_alpha = 0.35
        self.detection_mode = "Unknown"
        self.last_display_lanes = []
        self.last_display_fit_lines = []
        self.last_display_center_line = None
        self.last_raw_display_lanes = []
        self.last_detected_lane_roles = []
        self.last_yolo_debug = {
            'left_mask': False,
            'right_mask': False,
            'left_pts': 0,
            'right_pts': 0,
            'candidates': 0,
            'width_px': None,
            'valid_pair': False,
        }
        self.last_motion_text = "WAITING"
        self.last_stage_name = "INIT"
        self.last_extract_reason = "INIT"
        self.workflow_status_text = "ALIGNING"
        self.workflow_completed_cycles = 0
        self.force_realign_move_pending = False
        self.workflow_busy = False
        self.workflow_finished = False
        self.workflow_failed = False
        self.workflow_thread = None
        self.workflow_lock = threading.Lock()
        self.done_lock = threading.Lock()
        self.done_recv_count = 0
        self.done_last_value = None
        self.done_last_stamp = None
        self.stats = {
            'total_frames': 0,
            'control_cycles': 0,
            'successful_detections': 0,
            'total_movements': 0
        }

        # 初始化CAN总线
        try:
            self.can_bus = can.interface.Bus(
                channel='can0',
                bustype='socketcan',
                bitrate=1000000
            )
            self.can_bus.set_filters([
                {"can_id": self.can_id, "can_mask": 0x7FF, "extended": False}
            ])
            rospy.loginfo("✅ CAN总线初始化成功(已设置内核过滤器:ID=0x00A)")
        except Exception as e:
            rospy.logerr(f"CAN总线初始化失败: {e}. 模拟模式")
            self.can_bus = None

        self.flag_pub = rospy.Publisher(self.flag_topic, Int32, queue_size=1, latch=False)
        self.done_sub = rospy.Subscriber(self.done_topic, Int32, self.done_callback, queue_size=8)

        self.running = False
        self.control_thread = None
        rospy.loginfo("✅ 系统初始化完成")
        rospy.loginfo(
            f"✅ 工作流: enable={self.workflow_enable_auto}, flag_topic={self.flag_topic}, "
            f"done_topic={self.done_topic}, cycle_count={self.workflow_cycle_count}"
        )

    # ========== 应用层指令过滤函数 ==========
    def recv_cmd(self, cmd, timeout=0.3):
        """应用层过滤: 只返回 data[0] == cmd 的 CAN 帧"""
        if self.can_bus is None:
            mock_data = [cmd, 0, 0, 0, 0, 0, 0, 0]
            return type('MockMsg', (), {
                'arbitration_id': self.can_id,
                'data': mock_data
            })()

        start_time = time.time()
        discarded_count = 0

        while time.time() - start_time < timeout:
            msg = self.can_bus.recv(timeout=0.05)

            if msg is None:
                continue

            if msg.arbitration_id != self.can_id:
                discarded_count += 1
                continue

            if len(msg.data) == 0:
                discarded_count += 1
                continue

            if msg.data[0] == cmd:
                if self.DEBUG_MODE and discarded_count > 0:
                    rospy.logdebug(f"   [recv_cmd] 丢弃{discarded_count}个无关帧后找到0x{cmd:02X}")
                return msg
            else:
                discarded_count += 1
                continue

        if self.DEBUG_MODE:
            rospy.logwarn(f"   [recv_cmd] 超时{timeout:.2f}s, 未收到0x{cmd:02X}")
        return None

    # ========== 查询系统状态 ==========
    def query_system_status(self, retry_count=3):
        """查询系统状态"""
        if self.can_bus is None:
            return {
                'angle': 0,
                'speed': 0,
                'mode': self.MODE_LATERAL,
                'arrived': True,
                'motion_state': 0x00
            }

        for attempt in range(retry_count):
            try:
                while True:
                    msg = self.can_bus.recv(timeout=0.01)
                    if msg is None:
                        break

                query_msg = can.Message(
                    arbitration_id=self.can_id,
                    data=[0x26, 0, 0, 0, 0, 0, 0, 0],
                    is_extended_id=False
                )
                self.can_bus.send(query_msg)

                msg = self.recv_cmd(0x26, timeout=0.3)

                if msg is None:
                    if attempt < retry_count - 1:
                        time.sleep(0.1)
                        continue
                    else:
                        return None

                return {
                    'angle': (msg.data[1] << 8) | msg.data[2],
                    'speed': (msg.data[3] << 8) | msg.data[4],
                    'mode': msg.data[5],
                    'arrived': (msg.data[6] == 0x01),
                    'motion_state': msg.data[7]
                }

            except Exception as e:
                if attempt == retry_count - 1:
                    rospy.logwarn(f"   [0x26] 查询异常: {e}")

        return None

    # ========== 统一的模式切换函数 ==========
    def switch_mode_if_needed(self, target_mode):
        """统一的模式切换函数"""
        mode_names = {
            0x02: "二轮(0x02)",
            0x03: "角度(0x03)",
            0x04: "横移(0x04)"
        }
        mode_name = mode_names.get(target_mode, f"未知(0x{target_mode:02X})")

        try:
            rospy.loginfo(f"   [步骤1] 查询当前模式...")
            status = self.query_system_status()

            if status is None:
                rospy.logwarn("   ⚠️ 无法查询系统状态")
                return False

            current_mode = status['mode']
            rospy.loginfo(f"   当前模式: 0x{current_mode:02X}")

            if current_mode == target_mode:
                rospy.loginfo(f"   ✓ 已在目标模式 {mode_name}")
                self._update_current_mode(target_mode)
                return True

            rospy.loginfo(f"   [步骤2] 切换模式: 0x{current_mode:02X} → {mode_name}")

            if self.can_bus:
                msg = can.Message(
                    arbitration_id=self.can_id,
                    data=[0x05, 0, 0, target_mode, 0, 0, 0, 0],
                    is_extended_id=False
                )
                self.can_bus.send(msg)
                rospy.loginfo(f"   [0x05] 已发送切换命令: 05 00 00 {target_mode:02X} 00 00 00 00")

            rospy.loginfo(f"   等待{self.MODE_SWITCH_WAIT_TIME:.1f}秒...")
            time.sleep(self.MODE_SWITCH_WAIT_TIME)

            rospy.loginfo(f"   [步骤3] 等待切换完成")

            start_time = time.time()
            query_count = 0
            mode_matched = False
            arrived_count = 0
            required_count = 2

            while time.time() - start_time < self.QUERY_TIMEOUT:
                status = self.query_system_status()
                query_count += 1

                if status is None:
                    time.sleep(self.MODE_SWITCH_QUERY_INTERVAL)
                    continue

                if not mode_matched:
                    if status['mode'] == target_mode:
                        mode_matched = True
                        rospy.loginfo(f"      ✓ 模式位已变更 → {mode_name}")
                    else:
                        time.sleep(self.MODE_SWITCH_QUERY_INTERVAL)
                        continue

                if status['arrived']:
                    arrived_count += 1
                    if arrived_count >= required_count:
                        elapsed = time.time() - start_time
                        rospy.loginfo(f"   ✅ 模式切换完成 (耗时{elapsed:.2f}s)")
                        self._update_current_mode(target_mode)
                        return True
                else:
                    arrived_count = 0

                time.sleep(self.MODE_SWITCH_QUERY_INTERVAL)

            rospy.logwarn(f"   ⚠️ 模式切换超时")
            return False

        except Exception as e:
            rospy.logerr(f"❌ 模式切换异常: {e}")
            return False

    def _update_current_mode(self, target_mode):
        """更新当前模式记录"""
        if target_mode == self.MODE_LATERAL:
            self.current_mode = 'lateral'

    # ========== 【修复】智能运动监控 - 确保运动开始后再判断完成 ==========
    def wait_motion_complete(self, timeout=15.0, expected_duration=None):
        """
        等待运动完成 - 修复版
        
        关键修复:
        1. 先等待运动开始(speed > 0)
        2. 运动开始后再等待运动结束(speed == 0)
        3. 增加最小等待时间,避免过早判断完成
        """
        start_time = time.time()
        motion_started = False
        stable_count = 0
        required_stable_count = 3  # 需要连续3次speed=0才算完成
        query_count = 0
        max_speed_seen = 0
        
        # 最小等待时间: 根据预期运动时间设置,至少等待1秒
        min_wait_time = 1.0
        if expected_duration:
            min_wait_time = max(1.0, expected_duration * 0.5)

        rospy.loginfo(f"   [等待运动完成] 最小等待={min_wait_time:.1f}s, 超时={timeout:.1f}s")
        
        # ========== 阶段1: 等待运动开始 ==========
        rospy.loginfo(f"   [阶段1] 等待运动开始...")
        motion_start_timeout = 3.0  # 运动开始超时时间
        motion_start_time = time.time()
        
        while time.time() - motion_start_time < motion_start_timeout:
            status = self.query_system_status()
            query_count += 1
            
            if status is None:
                time.sleep(0.1)
                continue
            
            speed = status['speed']
            
            if speed > 0:
                motion_started = True
                max_speed_seen = max(max_speed_seen, speed)
                rospy.loginfo(f"   ✓ 运动已开始! speed={speed}")
                break
            
            # 即使speed=0,也可能是运动还没开始,继续等待
            time.sleep(0.15)
        
        if not motion_started:
            rospy.logwarn(f"   ⚠️ 运动未能在{motion_start_timeout}s内开始")
            # 即使没检测到运动开始,也要等待最小时间
            remaining_min_wait = min_wait_time - (time.time() - start_time)
            if remaining_min_wait > 0:
                rospy.loginfo(f"   等待最小时间 {remaining_min_wait:.1f}s...")
                time.sleep(remaining_min_wait)
            return True  # 可能是运动太快或查询不及时,假定完成
        
        # ========== 阶段2: 等待运动完成 ==========
        rospy.loginfo(f"   [阶段2] 等待运动完成...")
        
        while time.time() - start_time < timeout:
            status = self.query_system_status()
            query_count += 1
            
            if status is None:
                time.sleep(self.MOTION_QUERY_INTERVAL)
                continue
            
            speed = status['speed']
            max_speed_seen = max(max_speed_seen, speed)
            
            if speed == 0:
                # 检查是否已经过了最小等待时间
                elapsed = time.time() - start_time
                if elapsed < min_wait_time:
                    # 还没到最小等待时间,可能是误判
                    rospy.logdebug(f"      speed=0 但未到最小等待时间 ({elapsed:.1f}s < {min_wait_time:.1f}s)")
                    time.sleep(self.MOTION_QUERY_INTERVAL)
                    continue
                
                stable_count += 1
                if stable_count >= required_stable_count:
                    elapsed = time.time() - start_time
                    rospy.loginfo(f"   ✅ 运动完成! (耗时{elapsed:.2f}s, 最大速度={max_speed_seen})")
                    return True
            else:
                stable_count = 0  # 还在运动,重置计数
                if query_count % 5 == 0:
                    rospy.loginfo(f"      运动中: speed={speed}, 已耗时{time.time()-start_time:.1f}s")
            
            time.sleep(self.MOTION_QUERY_INTERVAL)
        
        elapsed = time.time() - start_time
        rospy.logwarn(f"   ⚠️ 运动超时 ({elapsed:.1f}s), 最大速度={max_speed_seen}")
        return False

    # ========== 【修复】横移运动完成等待 ==========
    def wait_lateral_motion_complete(self, distance_mm, speed_hall, timeout=15.0):
        """
        专门用于横移运动的等待函数
        """
        # 估算运动时间: distance_hall / speed_hall
        distance_hall = UnitConverter.mm_to_hall(abs(distance_mm))
        distance_mm_actual = UnitConverter.hall_to_mm(distance_hall)
        estimated_time = distance_hall / speed_hall if speed_hall > 0 else 5.0
        estimated_time += 1.0  # 余量
        
        rospy.loginfo(
            f"   [横移运动] 目标={distance_mm:.1f}mm -> {distance_hall}hall "
            f"(约{distance_mm_actual:.1f}mm), 速度={speed_hall}hall/s"
        )
        rospy.loginfo(f"   [横移运动] 预估时间={estimated_time:.1f}s")
        
        return self.wait_motion_complete(timeout=timeout, expected_duration=estimated_time)

    def send_lateral_position_command(self, distance_mm, direction, speed_hall):
        """发送横移位置命令 (0x0E)"""
        distance_abs = abs(distance_mm)
        if distance_abs < 1e-6:
            direction_text = "停止保持"
        else:
            direction_text = "向右" if direction > 0 else "向左"
        self.last_motion_text = f"LATERAL {distance_abs:.1f}mm {direction_text} @ {speed_hall}hall/s"

        if self.can_bus is None:
            rospy.loginfo(f"   >>> 执行横移命令[模拟]: {self.last_motion_text}")
            return

        distance_hall = UnitConverter.mm_to_hall(distance_abs)
        distance_mm_actual = UnitConverter.hall_to_mm(distance_hall)
        direction_byte = 0x01 if direction > 0 else 0x02

        data = bytearray(8)
        data[0] = 0x0E
        data[1] = (distance_hall >> 8) & 0xFF
        data[2] = distance_hall & 0xFF
        data[3] = (speed_hall >> 8) & 0xFF
        data[4] = speed_hall & 0xFF
        data[5] = direction_byte

        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        self.can_bus.send(msg)

        rospy.loginfo(
            f"   >>> 执行横移命令: {self.last_motion_text} "
            f"({distance_hall}hall≈{distance_mm_actual:.1f}mm, dir=0x{direction_byte:02X})"
        )
        rospy.loginfo(f"   [0x0E] 数据: {' '.join(f'{b:02X}' for b in data)}")

    def pixel_offset_to_lateral_direction(self, pixel_offset):
        """像素偏差映射为横移方向: 负偏差向左, 正偏差向右。"""
        pixel_offset = float(pixel_offset)
        if abs(pixel_offset) <= 1e-6:
            return 0
        raw_direction = 1 if pixel_offset > 0 else -1
        mapped = raw_direction * self.pixel_direction_sign
        return 1 if mapped > 0 else -1

    def stop_motion(self):
        """停止运动"""
        self.last_motion_text = "STOP"
        rospy.loginfo("⏹️ 停止运动")
        if self.current_mode == 'lateral':
            self.last_motion_text = "STOP(lateral mode)"

    def done_callback(self, msg):
        value = int(msg.data)
        with self.done_lock:
            self.done_recv_count += 1
            self.done_last_value = value
            self.done_last_stamp = time.time()
            recv_count = self.done_recv_count

        result = "成功" if value > 0 else "失败"
        rospy.loginfo(f"工作流收到完成信号: value={value}, result={result}, recv_count={recv_count}")

    def snapshot_done_count(self):
        with self.done_lock:
            return self.done_recv_count

    def wait_next_done(self, previous_count, timeout_sec):
        rate = rospy.Rate(20)

        timeout_sec = float(timeout_sec)
        if timeout_sec <= 0.0:
            while not rospy.is_shutdown():
                with self.done_lock:
                    if self.done_recv_count > previous_count:
                        return self.done_last_value
                rate.sleep()
            return None

        deadline = time.time() + max(0.1, timeout_sec)
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.done_lock:
                if self.done_recv_count > previous_count:
                    return self.done_last_value
            rate.sleep()

        return None

    def publish_flag_burst(self, value, duration_sec, log_prefix):
        msg = Int32(data=int(value))
        duration_sec = max(0.1, float(duration_sec))
        rate_hz = max(1.0, float(self.workflow_flag_rate_hz))
        rate = rospy.Rate(rate_hz)
        end_time = time.time() + duration_sec
        publish_count = 0

        rospy.loginfo(
            f"{log_prefix}: 开始发送标志位 value={msg.data}, "
            f"duration={duration_sec:.1f}s, rate={rate_hz:.1f}Hz"
        )

        while not rospy.is_shutdown() and time.time() < end_time:
            self.flag_pub.publish(msg)
            publish_count += 1
            rate.sleep()

        rospy.loginfo(f"{log_prefix}: 标志位发送结束, 共发送 {publish_count} 次")

    def reset_alignment_cycle_state(self, reason):
        rospy.loginfo(f"工作流: {reason}，准备重新进入对轨检测")
        self.stop_motion()
        self.frame_buffer.clear()
        self.stable_lateral_error_mm = 0
        self.last_pixel_offset_px = 0.0
        self.last_rail_center_x = self.img_width / 2.0
        self.last_target_center_x = self.pixel_lateral_calculator.target_center_x
        self.last_rail_width_px = 0.0
        self.last_step_mm = 0.0
        self.last_stage_name = 'INIT'
        self.last_motion_text = 'WAITING'
        self.last_extract_reason = 'RESET_FOR_NEXT_CYCLE'
        self.last_accepted_pixel_offset_px = None
        self.last_accepted_lateral_error_mm = None
        self.fitted_lines_pixel = []
        self.yolo_expected_left_x = None
        self.yolo_expected_right_x = None
        self.yolo_prev_left_line = None
        self.yolo_prev_right_line = None
        self.yolo_prev_gauge_px = None
        self.last_display_fit_lines = []
        self.force_realign_move_pending = bool(self.force_realign_after_cycle_shift)
        if self.force_realign_move_pending:
            rospy.loginfo(
                f"工作流: 已开启下一轮强制横移对齐，至少发送一次位置命令 "
                f"(min_px={self.force_realign_min_px:.1f})"
            )
        with self.state_lock:
            self.current_state = SystemState.IDLE

    def maybe_start_workflow_cycle(self):
        if not self.workflow_enable_auto:
            return

        with self.workflow_lock:
            if self.workflow_finished or self.workflow_busy:
                return
            if self.workflow_cycle_count > 0 and self.workflow_completed_cycles >= self.workflow_cycle_count:
                self.workflow_finished = True
                self.workflow_status_text = "COMPLETE"
                return

            cycle_index = self.workflow_completed_cycles + 1
            self.workflow_busy = True
            self.workflow_status_text = f"TRIGGER_{cycle_index}"

        self.workflow_thread = threading.Thread(
            target=self.execute_external_cycle_workflow,
            args=(cycle_index,),
            daemon=True,
        )
        self.workflow_thread.start()

    def execute_external_cycle_workflow(self, cycle_index):
        success = False
        try:
            deadline = time.time() + max(0.0, self.workflow_wait_subscriber_sec)
            while time.time() < deadline and not rospy.is_shutdown():
                if self.flag_pub.get_num_connections() > 0:
                    break
                rospy.sleep(0.05)

            rospy.loginfo(f"工作流第 {cycle_index} 轮: 对轨完成，开始触发上轨往返流程")
            done_snapshot = self.snapshot_done_count()
            self.publish_flag_burst(
                self.workflow_trigger_value,
                self.workflow_flag_duration_sec,
                log_prefix=f"工作流第 {cycle_index} 轮触发",
            )

            with self.workflow_lock:
                self.workflow_status_text = f"WAIT_DONE_{cycle_index}"

            if self.workflow_done_timeout_sec > 0:
                rospy.loginfo(
                    f"工作流第 {cycle_index} 轮: 等待完成信号, timeout={self.workflow_done_timeout_sec:.1f}s"
                )
            else:
                rospy.loginfo(f"工作流第 {cycle_index} 轮: 等待完成信号, timeout=disabled(持续等待)")

            done_value = self.wait_next_done(done_snapshot, self.workflow_done_timeout_sec)
            if done_value is None:
                if rospy.is_shutdown():
                    rospy.logwarn(f"工作流第 {cycle_index} 轮: 节点关闭，结束等待完成信号")
                else:
                    rospy.logerr(f"工作流第 {cycle_index} 轮: 等待完成信号超时")
                with self.workflow_lock:
                    self.workflow_failed = True
                    self.workflow_finished = True
                    self.workflow_status_text = "FAILED_TIMEOUT" if not rospy.is_shutdown() else "STOPPED"
                return

            if done_value < 0:
                rospy.logerr(f"工作流第 {cycle_index} 轮: 收到失败完成信号 {done_value}")
                with self.workflow_lock:
                    self.workflow_failed = True
                    self.workflow_status_text = f"FAILED_{cycle_index}"
                    if self.workflow_stop_on_failure:
                        self.workflow_finished = True
                if self.workflow_stop_on_failure:
                    return

            success = done_value > 0
            if success:
                with self.workflow_lock:
                    self.workflow_completed_cycles += 1
                    completed_cycles = self.workflow_completed_cycles
                self.publish_flag_burst(
                    0,
                    self.workflow_flag_reset_duration_sec,
                    log_prefix=f"工作流第 {cycle_index} 轮清零",
                )

                if self.workflow_cycle_count == 0 or completed_cycles < self.workflow_cycle_count:
                    if self.workflow_inter_cycle_delay_sec > 0:
                        rospy.loginfo(
                            f"工作流第 {cycle_index} 轮: 等待 {self.workflow_inter_cycle_delay_sec:.1f}s 后重新对轨"
                        )
                        rospy.sleep(self.workflow_inter_cycle_delay_sec)
                    self.reset_alignment_cycle_state(
                        reason=f"第 {cycle_index} 轮上轨/下轨/平移已完成"
                    )
                    with self.workflow_lock:
                        self.workflow_status_text = f"ALIGNING_{completed_cycles + 1}"
                else:
                    rospy.loginfo("工作流全部轮次完成")
                    with self.workflow_lock:
                        self.workflow_finished = True
                        self.workflow_status_text = "COMPLETE"
        finally:
            with self.workflow_lock:
                self.workflow_busy = False

    # ========== 轨道检测 ==========
    def fit_line_pixel(self, points):
        """像素坐标系下拟合直线"""
        if len(points) < 2:
            return None
        points = np.array(points)
        x_coords = points[:, 0]
        y_coords = points[:, 1]

        mask = y_coords > self.img_height * 0.3
        if np.sum(mask) < 2:
            return None

        x_valid = x_coords[mask]
        y_valid = y_coords[mask]
        weights = y_valid / self.img_height

        try:
            coeffs = np.polyfit(y_valid, x_valid, deg=1, w=weights)
            return coeffs
        except:
            return None

    def _estimate_lane_pair_width_px(self, left_pts, right_pts):
        """按共同采样行估计双轨像素轨距,允许左右点数不一致。"""
        if left_pts is None or right_pts is None:
            return None

        left_pts = np.asarray(left_pts, dtype=np.float32).reshape(-1, 2)
        right_pts = np.asarray(right_pts, dtype=np.float32).reshape(-1, 2)
        if len(left_pts) == 0 or len(right_pts) == 0:
            return None

        left_rows = {int(round(float(y))): float(x) for x, y in left_pts}
        matched_widths = []
        for x, y in right_pts:
            y_key = int(round(float(y)))
            if y_key in left_rows:
                matched_widths.append(float(x) - left_rows[y_key])

        min_matches = max(3, self.yolo_min_points // 2)
        if len(matched_widths) >= min_matches:
            return float(np.median(matched_widths))

        left_sorted = left_pts[np.argsort(left_pts[:, 1])]
        right_sorted = right_pts[np.argsort(right_pts[:, 1])]
        y_min = max(float(left_sorted[0, 1]), float(right_sorted[0, 1]))
        y_max = min(float(left_sorted[-1, 1]), float(right_sorted[-1, 1]))
        if y_max <= y_min:
            return None

        sample_count = int(min(len(left_sorted), len(right_sorted), 15))
        if sample_count < 2:
            return None

        y_grid = np.linspace(y_min, y_max, sample_count, dtype=np.float32)
        left_x = np.interp(y_grid, left_sorted[:, 1], left_sorted[:, 0])
        right_x = np.interp(y_grid, right_sorted[:, 1], right_sorted[:, 0])
        return float(np.median(right_x - left_x))

    def _extract_valid_lanes_from_yolo(self, result, update_expected=True):
        valid_lanes = []
        self.last_raw_display_lanes = []
        self.last_detected_lane_roles = []
        self.last_yolo_debug = {
            'left_mask': False,
            'right_mask': False,
            'left_pts': 0,
            'right_pts': 0,
            'candidates': 0,
            'width_px': None,
            'valid_pair': False,
        }
        if result is None or getattr(result, 'masks', None) is None:
            return valid_lanes

        min_y = max(self.yolo_min_y, int(self.img_height * 0.3))
        y_grid = np.arange(min_y, self.img_height, self.yolo_row_step, dtype=np.float32)
        left_mask = choose_best_mask(
            result,
            cls_id=0,
            min_y=min_y,
            side_prior=self.yolo_side_prior,
            side_margin=self.yolo_side_margin,
            expected_x=self.yolo_expected_left_x,
        )
        right_mask = choose_best_mask(
            result,
            cls_id=1,
            min_y=min_y,
            side_prior=self.yolo_side_prior,
            side_margin=self.yolo_side_margin,
            expected_x=self.yolo_expected_right_x,
        )

        self.last_yolo_debug['left_mask'] = left_mask is not None
        self.last_yolo_debug['right_mask'] = right_mask is not None
        left_pts_raw = []
        right_pts_raw = []
        if left_mask is not None:
            left_pts_raw = sample_line_points(left_mask, min_y, self.yolo_row_step)
            self.last_yolo_debug['left_pts'] = len(left_pts_raw)

        if right_mask is not None:
            right_pts_raw = sample_line_points(right_mask, min_y, self.yolo_row_step)
            self.last_yolo_debug['right_pts'] = len(right_pts_raw)

        left_line = fit_line_from_points(
            left_pts_raw,
            y_grid,
            residual_thr=self.yolo_fit_residual,
            min_points=self.yolo_min_points,
        ) if len(left_pts_raw) > 0 else None
        right_line = fit_line_from_points(
            right_pts_raw,
            y_grid,
            residual_thr=self.yolo_fit_residual,
            min_points=self.yolo_min_points,
        ) if len(right_pts_raw) > 0 else None

        if update_expected:
            if left_line is not None:
                self.yolo_prev_left_line = left_line
            if right_line is not None:
                self.yolo_prev_right_line = right_line
            if left_line is not None and right_line is not None:
                self.yolo_prev_gauge_px = float(np.median(right_line - left_line))
            if left_line is not None:
                self.yolo_expected_left_x = float(left_line[-1])
            if right_line is not None:
                self.yolo_expected_right_x = float(right_line[-1])

        candidates = []
        if left_line is not None:
            left_pts = line_to_points(left_line, y_grid, self.img_width)
            if len(left_pts) >= 2:
                candidates.append(("left", np.asarray(left_pts, dtype=np.float32)))
        if right_line is not None:
            right_pts = line_to_points(right_line, y_grid, self.img_width)
            if len(right_pts) >= 2:
                candidates.append(("right", np.asarray(right_pts, dtype=np.float32)))

        self.last_yolo_debug['candidates'] = len(candidates)
        if len(candidates) == 0:
            return valid_lanes
        if len(candidates) == 1:
            self.last_raw_display_lanes = [np.asarray(candidates[0][1], dtype=np.float32)]
            self.last_detected_lane_roles = [candidates[0][0]]
            return [candidates[0][1]]

        # 两阶段拟合输入: 先按左右语义筛选，再按图像位置稳定排序。
        left_pts = candidates[0][1]
        right_pts = candidates[1][1]
        if float(np.median(left_pts[:, 0])) > float(np.median(right_pts[:, 0])):
            left_pts, right_pts = right_pts, left_pts

        self.last_raw_display_lanes = [
            np.asarray(left_pts, dtype=np.float32),
            np.asarray(right_pts, dtype=np.float32),
        ]

        width_px = self._estimate_lane_pair_width_px(left_pts, right_pts)
        self.last_yolo_debug['width_px'] = width_px
        if width_px is not None and 80.0 <= width_px <= 1100.0:
            self.last_yolo_debug['valid_pair'] = True
            self.last_detected_lane_roles = ["left", "right"]
            return [left_pts, right_pts]
        self.last_detected_lane_roles = ["left"]
        return [left_pts]

    def _extract_valid_lanes(self, detection_result):
        """提取 YOLO 有效轨道点。"""
        return self._extract_valid_lanes_from_yolo(detection_result)

    def _update_display_cache_from_lanes(self, lane_list):
        self.last_display_lanes = [np.asarray(lane_points, dtype=np.float32).reshape(-1, 2) for lane_points in lane_list]
        self.last_display_fit_lines = []
        self.last_display_center_line = None
        for lane_points in self.last_display_lanes:
            coeffs = self.fit_line_pixel(lane_points)
            if coeffs is not None:
                self.last_display_fit_lines.append(coeffs)
        if len(self.last_display_fit_lines) >= 2:
            left_line = self.last_display_fit_lines[0]
            right_line = self.last_display_fit_lines[1]
            self.last_display_center_line = np.asarray(
                [
                    (float(left_line[0]) + float(right_line[0])) / 2.0,
                    (float(left_line[1]) + float(right_line[1])) / 2.0,
                ],
                dtype=np.float32,
            )
        elif len(self.last_display_fit_lines) == 1 and len(self.last_detected_lane_roles) == 1:
            role = self.last_detected_lane_roles[0]
            gauge_px = self.yolo_prev_gauge_px
            if gauge_px is None:
                gauge_px = self.default_single_lane_gauge_px
            if role in ("left", "right") and gauge_px is not None and 80.0 <= float(gauge_px) <= 1100.0:
                k, b = self.last_display_fit_lines[0]
                shift = float(gauge_px) / 2.0
                if role == "left":
                    center_b = float(b) + shift
                else:
                    center_b = float(b) - shift
                self.last_display_center_line = np.asarray([float(k), center_b], dtype=np.float32)

    def update_live_display_cache(self, detection_result):
        valid_lanes = self._extract_valid_lanes_from_yolo(detection_result, update_expected=False)

        if len(self.last_raw_display_lanes) > 0:
            self._update_display_cache_from_lanes(self.last_raw_display_lanes)
        else:
            self._update_display_cache_from_lanes(valid_lanes)

    def extract_pixel_lines_only(self, detection_result):
        """
        仅提取像素拟合线 (用于角度补偿后的重采集)
        不进行IPM转换和误差计算
        """
        valid_lanes = self._extract_valid_lanes(detection_result)
        if len(self.last_raw_display_lanes) > 0:
            self._update_display_cache_from_lanes(self.last_raw_display_lanes)
        else:
            self._update_display_cache_from_lanes(valid_lanes)
        if len(valid_lanes) == 0:
            return None

        pixel_lines = []
        for lane_points in valid_lanes:
            pixel_coeffs = self.fit_line_pixel(lane_points)
            if pixel_coeffs is not None:
                pixel_lines.append(pixel_coeffs)

        # 更新检测模式
        if len(valid_lanes) >= 2:
            self.detection_mode = "Dual-Rail"
        else:
            self.detection_mode = "Single-Rail"

        return pixel_lines if len(pixel_lines) > 0 else None

    def extract_pixel_measurement(self, detection_result):
        pixel_lines = self.extract_pixel_lines_only(detection_result)
        if pixel_lines is None or len(pixel_lines) == 0:
            dbg = self.last_yolo_debug
            self.last_extract_reason = (
                f"no_pixel_lines Lm={int(dbg['left_mask'])}/Lpts={dbg['left_pts']} "
                f"Rm={int(dbg['right_mask'])}/Rpts={dbg['right_pts']} "
                f"cand={dbg['candidates']} width={dbg['width_px']}"
            )
            return None

        measure = self.pixel_lateral_calculator.measure_from_pixel(
            pixel_lines,
            self.detection_mode,
            lane_roles=self.last_detected_lane_roles,
            fallback_gauge_px=self.yolo_prev_gauge_px,
            default_gauge_px=self.default_single_lane_gauge_px,
        )
        if measure is None:
            gauge_text = "None" if self.yolo_prev_gauge_px is None else f"{float(self.yolo_prev_gauge_px):.1f}"
            self.last_extract_reason = (
                f"pixel_measure_failed_{self.detection_mode}"
                f"_roles={self.last_detected_lane_roles}_gauge={gauge_text}"
            )
            return None

        self.fitted_lines_pixel = pixel_lines
        if measure.get('inferred_from_single', False):
            self.last_extract_reason = (
                f"ok_pixel_{self.detection_mode}_single_with_gauge_{float(measure['rail_width_px']):.1f}"
            )
        else:
            self.last_extract_reason = f"ok_pixel_{self.detection_mode}"
        return measure

    def filter_collected_data(self):
        """对采集到的像素偏差做中值滤波"""
        if len(self.frame_buffer) == 0:
            return None

        valid_measures = [item for item in self.frame_buffer if item is not None]
        if len(valid_measures) == 0:
            dbg = self.last_yolo_debug
            rospy.logwarn(
                "⚠️ 本轮收集成功帧=0/"
                f"{len(self.frame_buffer)} | "
                f"Lm={int(dbg['left_mask'])} Lpts={dbg['left_pts']} "
                f"Rm={int(dbg['right_mask'])} Rpts={dbg['right_pts']} "
                f"cand={dbg['candidates']} width={dbg['width_px']} "
                f"reason={self.last_extract_reason}"
            )
            return None

        pixel_median = float(np.median([item['pixel_offset_px'] for item in valid_measures]))
        center_median = float(np.median([item['rail_center_x'] for item in valid_measures]))
        target_median = float(np.median([item['target_center_x'] for item in valid_measures]))
        width_median = float(np.median([item['rail_width_px'] for item in valid_measures]))
        lateral_median = float(np.median([item['lateral_mm'] for item in valid_measures]))

        if self.last_accepted_pixel_offset_px is not None:
            pixel_jump = abs(pixel_median - self.last_accepted_pixel_offset_px)
            if pixel_jump > self.max_pixel_jump_px:
                rospy.logwarn(f"⚠️ 像素偏差跳变过大, 启用平滑: Δpixel={pixel_jump:.1f}px")
                pixel_median = (
                    self.smoothing_alpha * pixel_median +
                    (1.0 - self.smoothing_alpha) * self.last_accepted_pixel_offset_px
                )
                lateral_median = pixel_median * self.pixel_lateral_calculator.PIXEL_TO_MM_RATIO

        self.last_accepted_pixel_offset_px = pixel_median
        self.last_accepted_lateral_error_mm = lateral_median

        return {
            'pixel_offset_px': pixel_median,
            'abs_pixel_offset_px': abs(pixel_median),
            'rail_center_x': center_median,
            'target_center_x': target_median,
            'rail_width_px': width_median,
            'lateral_mm': lateral_median,
            'direction': 1 if pixel_median >= 0 else -1,
            'valid_count': len(valid_measures),
        }

    def apply_measurement(self, measure):
        """把当前帧或滤波后的测量结果写回控制状态。"""
        if measure is None:
            return
        self.last_pixel_offset_px = float(measure['pixel_offset_px'])
        self.last_rail_center_x = float(measure['rail_center_x'])
        self.last_target_center_x = float(measure['target_center_x'])
        self.last_rail_width_px = float(measure['rail_width_px'])
        self.stable_lateral_error_mm = float(measure['lateral_mm'])

    def pixel_error_to_step_mm(self, abs_pixel_offset):
        abs_pixel_offset = abs(float(abs_pixel_offset))
        if abs_pixel_offset > self.pixel_deadband_px:
            if abs_pixel_offset < float(self.pixel_small_error_threshold_px):
                return float(self.pixel_small_error_step_mm)
            return float(self.pixel_step_mm)
        return 0.0

    def control_loop(self):
        """仅基于像素偏差做横移位置逼近，不再做角度/IPM补偿。"""
        while self.running:
            try:
                with self.state_lock:
                    current_state = self.current_state

                if current_state == SystemState.PROCESSING:
                    self.stats['control_cycles'] += 1
                    abs_px = abs(self.last_pixel_offset_px)
                    force_move_this_cycle = (
                        self.force_realign_move_pending and
                        abs_px >= self.force_realign_min_px
                    )
                    self.last_stage_name = 'PIXEL_LATERAL'

                    rospy.loginfo(f"\n{'=' * 70}")
                    rospy.loginfo(f"   控制周期 #{self.stats['control_cycles']}")
                    rospy.loginfo(f"   当前像素偏差: {self.last_pixel_offset_px:+.1f}px")
                    rospy.loginfo(f"   当前轨道中心: {self.last_rail_center_x:.1f}px")
                    rospy.loginfo(f"   当前目标中心: {self.last_target_center_x:.1f}px")
                    rospy.loginfo(f"   当前轨宽估计: {self.last_rail_width_px:.1f}px")
                    rospy.loginfo(f"{'=' * 70}")

                    if abs_px <= self.pixel_deadband_px and not force_move_this_cycle:
                        self.last_motion_text = "HOLD(pixel aligned)"
                        rospy.loginfo(f"✅ 像素对齐完成: {self.last_pixel_offset_px:+.1f}px")
                        with self.state_lock:
                            self.current_state = SystemState.ALIGNED
                        time.sleep(0.1)
                        continue

                    if not self.switch_mode_if_needed(self.MODE_LATERAL):
                        with self.state_lock:
                            self.current_state = SystemState.IDLE
                        continue

                    direction = self.pixel_offset_to_lateral_direction(self.last_pixel_offset_px)
                    move_mm = self.pixel_error_to_step_mm(abs_px)
                    if force_move_this_cycle and move_mm <= 0.0 and direction != 0:
                        if abs_px < float(self.pixel_small_error_threshold_px):
                            move_mm = float(self.pixel_small_error_step_mm)
                        else:
                            move_mm = float(self.pixel_step_mm)
                        rospy.loginfo(
                            f"   下一轮强制对齐已生效: 偏差={self.last_pixel_offset_px:+.1f}px "
                            f"位于死区内，仍执行一次 {move_mm:.1f}mm 横移"
                        )
                    if move_mm <= 0.0:
                        self.last_motion_text = "HOLD(pixel deadband)"
                        self.force_realign_move_pending = False
                        with self.state_lock:
                            self.current_state = SystemState.ALIGNED
                        continue

                    self.last_stage_name = 'PIXEL_POSITION'
                    speed = self.pixel_step_speed_hall
                    self.last_step_mm = move_mm
                    rospy.loginfo(
                        f"   位置模式: 偏差={self.last_pixel_offset_px:+.1f}px "
                        f"→ {move_mm:.1f}mm {'向右' if direction > 0 else '向左'} @ {speed}hall/s"
                    )
                    self.send_lateral_position_command(move_mm, direction, speed)
                    self.stats['total_movements'] += 1
                    self.force_realign_move_pending = False
                    self.wait_lateral_motion_complete(move_mm, speed, timeout=15.0)
                    time.sleep(self.pixel_step_settle_sec)
                    with self.state_lock:
                        self.current_state = SystemState.IDLE

                elif current_state == SystemState.ALIGNED:
                    time.sleep(0.2)
                else:
                    time.sleep(0.05)

            except Exception as e:
                rospy.logerr(f"❌ 控制循环错误: {e}")
                self.stop_motion()
                import traceback
                traceback.print_exc()

        self.stop_motion()
        rospy.loginfo("   控制线程结束")

    def _draw_lane_polyline(self, image, lane_points, color, label=None):
        pts = np.asarray(lane_points, dtype=np.float32).reshape(-1, 2)
        if len(pts) < 2:
            return
        poly = np.round(pts).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(image, [poly], False, color, 2, cv2.LINE_AA)
        if label:
            x0, y0 = poly[-1, 0]
            cv2.putText(image, label, (int(x0) + 8, max(20, int(y0) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def _draw_fitted_line(self, image, coeffs, color, label=None):
        if coeffs is None or len(coeffs) < 2:
            return
        k, b = coeffs
        y_start = max(self.yolo_min_y, int(self.img_height * 0.3))
        pts = []
        for y in range(y_start, self.img_height, self.yolo_row_step):
            x = k * y + b
            if 0 <= x < self.img_width:
                pts.append((int(round(x)), int(y)))
        if len(pts) < 2:
            return
        poly = np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(image, [poly], False, color, 2, cv2.LINE_AA)
        if label:
            x0, y0 = poly[0, 0]
            cv2.putText(image, label, (int(x0) + 8, min(self.img_height - 10, int(y0) + 24)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def _get_display_lane_styles(self):
        if len(self.last_display_lanes) == 1:
            lane = np.asarray(self.last_display_lanes[0], dtype=np.float32).reshape(-1, 2)
            if len(lane) > 0 and float(np.median(lane[:, 0])) > (self.img_width / 2.0):
                return [("RIGHT", (255, 0, 0))]
            return [("LEFT", (0, 0, 255))]

        return [
            ("LEFT", (0, 0, 255)),
            ("RIGHT", (255, 0, 0)),
            ("RAIL3", (0, 255, 255)),
        ]

    def create_visualization(self, frame, detection_result=None):
        """创建可视化界面"""
        if frame is None:
            return None
        vis_frame = frame.copy()

        lane_styles = self._get_display_lane_styles()
        for idx, lane_points in enumerate(self.last_display_lanes[:3]):
            if idx < len(lane_styles):
                label, color = lane_styles[idx]
            else:
                label, color = (f"RAIL{idx+1}", (0, 255, 255))
            self._draw_lane_polyline(vis_frame, lane_points, color, label)

        active_fit_lines = self.last_display_fit_lines
        for idx, coeffs in enumerate(active_fit_lines[:2]):
            color = lane_styles[idx][1] if idx < len(lane_styles) else (0, 255, 255)
            self._draw_fitted_line(vis_frame, coeffs, color, None)
        if self.last_display_center_line is not None:
            self._draw_fitted_line(vis_frame, self.last_display_center_line, (0, 255, 0), "CENTER")

        panel = vis_frame.copy()
        cv2.rectangle(panel, (10, 10), (1180, 220), (0, 0, 0), -1)
        vis_frame = cv2.addWeighted(panel, 0.28, vis_frame, 0.72, 0)

        with self.state_lock:
            state = self.current_state

        info_y = 34
        cv2.putText(vis_frame, f"Stage: {self.last_stage_name}", (22, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 220, 0), 2)
        info_y += 30
        cv2.putText(
            vis_frame,
            f"Pixel: {self.last_pixel_offset_px:+.1f}px   Move: {self.stable_lateral_error_mm:+.1f}mm   Step: {self.last_step_mm:.1f}mm",
            (22, info_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (0, 255, 255),
            2,
        )
        info_y += 30
        cv2.putText(
            vis_frame,
            f"Center: {self.last_rail_center_x:.1f}px   Target: {self.last_target_center_x:.1f}px   Width: {self.last_rail_width_px:.1f}px",
            (22, info_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 220, 180),
            2,
        )
        info_y += 30
        cv2.putText(vis_frame, f"Lanes: {len(self.last_display_lanes)}   Mode: {self.detection_mode}   State: {state.name}",
                    (22, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        info_y += 30
        cv2.putText(vis_frame, f"Cmd: {self.last_motion_text}", (22, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
        info_y += 30
        workflow_total = "INF" if self.workflow_cycle_count == 0 else str(self.workflow_cycle_count)
        cv2.putText(
            vis_frame,
            f"Workflow: {self.workflow_status_text}   Cycle: {self.workflow_completed_cycles}/{workflow_total}",
            (22, info_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (120, 255, 180),
            2,
        )
        info_y += 30
        cv2.putText(vis_frame, f"Reason: {self.last_extract_reason}", (22, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 180, 0), 2)
        info_y += 30
        dbg = self.last_yolo_debug
        width_text = 'NA' if dbg['width_px'] is None else f"{dbg['width_px']:.1f}"
        cv2.putText(vis_frame,
                    f"YOLO: Lm={int(dbg['left_mask'])} Lpts={dbg['left_pts']}  Rm={int(dbg['right_mask'])} Rpts={dbg['right_pts']}  cand={dbg['candidates']} width={width_text}",
                    (22, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (180, 220, 255), 2)
        if len(self.last_display_lanes) == 1:
            info_y += 26
            cv2.putText(vis_frame, "Only one lane is currently usable", (22, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 140, 255), 2)

        screen_center_x = int(round(self.pixel_lateral_calculator.screen_center_x))
        target_center_x = int(round(self.last_target_center_x))
        cv2.line(vis_frame, (screen_center_x, 0), (screen_center_x, self.img_height), (100, 100, 100), 1)
        cv2.line(vis_frame, (target_center_x, 0), (target_center_x, self.img_height), (0, 180, 0), 2)
        roi_y = max(self.yolo_min_y, int(self.img_height * 0.3))
        cv2.line(vis_frame, (0, roi_y), (self.img_width - 1, roi_y), (255, 255, 255), 1)
        sample_y = int(self.img_height * 0.75)
        cv2.line(vis_frame, (0, sample_y), (self.img_width - 1, sample_y), (0, 255, 255), 1)

        return vis_frame

    @staticmethod
    def _build_gamma_lut(gamma):
        gamma = max(0.05, float(gamma))
        values = np.arange(256, dtype=np.float32) / 255.0
        lut = np.power(values, gamma) * 255.0
        return np.clip(lut, 0, 255).astype(np.uint8)

    def preprocess_frame_for_model(self, frame):
        if frame is None or not self.model_exposure_enable:
            return frame

        processed = frame
        input_mean = float(np.mean(processed))

        if self.model_exposure_clahe_clip > 0.0:
            lab = cv2.cvtColor(processed, cv2.COLOR_BGR2LAB)
            l_channel, a_channel, b_channel = cv2.split(lab)
            tile = max(2, int(self.model_exposure_clahe_tile))
            clahe = cv2.createCLAHE(
                clipLimit=max(0.1, float(self.model_exposure_clahe_clip)),
                tileGridSize=(tile, tile),
            )
            l_channel = clahe.apply(l_channel)
            processed = cv2.cvtColor(cv2.merge((l_channel, a_channel, b_channel)), cv2.COLOR_LAB2BGR)

        if abs(float(self.model_exposure_gamma) - 1.0) > 1e-3:
            processed = cv2.LUT(processed, self._model_exposure_gamma_lut)

        if (
            abs(float(self.model_exposure_alpha) - 1.0) > 1e-3
            or abs(float(self.model_exposure_beta)) > 1e-3
        ):
            processed = cv2.convertScaleAbs(
                processed,
                alpha=float(self.model_exposure_alpha),
                beta=float(self.model_exposure_beta),
            )

        now = time.time()
        if now - self._model_exposure_last_log_time >= max(1.0, self.model_exposure_log_interval_sec):
            self._model_exposure_last_log_time = now
            rospy.loginfo(f"🌙 模型前曝光增强: mean {input_mean:.1f}->{float(np.mean(processed)):.1f}")

        return processed

    def maybe_save_raw_frame(self, frame):
        if not self.raw_capture_enabled or self.raw_capture_session_dir is None or frame is None:
            return

        now = time.time()
        if (now - self.last_raw_capture_time) < self.raw_capture_interval_sec:
            return

        timestamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(now))
        millis = int((now - int(now)) * 1000)
        filename = f"raw_{timestamp}_{millis:03d}.jpg"
        path = os.path.join(self.raw_capture_session_dir, filename)
        ok = cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if ok:
            self.raw_capture_count += 1
            self.last_raw_capture_time = now
            rospy.loginfo(f"📷 原图抓拍[{self.raw_capture_count}]: {path}")
        else:
            rospy.logwarn(f"⚠️ 原图抓拍保存失败: {path}")

    # ========== 主运行函数 ==========
    def run(self):
        """主运行函数"""
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

                self.maybe_save_raw_frame(frame)
                self.stats['total_frames'] += 1
                model_frame = self.preprocess_frame_for_model(frame)
                detection_result = self.model.predict(
                    model_frame,
                    conf=self.yolo_conf,
                    imgsz=self.yolo_imgsz,
                    device=self.yolo_device,
                    verbose=False,
                    save=False,
                )[0]

                with self.state_lock:
                    current_state = self.current_state

                if current_state != SystemState.COLLECTING:
                    self.update_live_display_cache(detection_result)

                if not paused:
                    if current_state == SystemState.IDLE:
                        self.frame_buffer.clear()
                        self.last_stage_name = 'COLLECT'
                        with self.state_lock:
                            self.current_state = SystemState.COLLECTING

                    elif current_state == SystemState.COLLECTING:
                        measure = self.extract_pixel_measurement(detection_result)
                        if measure is not None:
                            self.apply_measurement(measure)
                            self.frame_buffer.append(measure)
                            inferred_text = " 推中线" if measure.get('inferred_from_single', False) else ""
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
                                self.stats['successful_detections'] += 1
                                with self.state_lock:
                                    self.current_state = SystemState.PROCESSING
                            else:
                                rospy.logwarn("⚠️ 滤波失败,重新收集")
                                with self.state_lock:
                                    self.current_state = SystemState.IDLE

                    with self.state_lock:
                        latest_state = self.current_state
                    if latest_state == SystemState.ALIGNED:
                        self.maybe_start_workflow_cycle()

                vis_frame = self.create_visualization(frame, detection_result)

                if paused:
                    cv2.putText(vis_frame, "PAUSED",
                                (self.img_width // 2 - 100, self.img_height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)

                cv2.namedWindow('Rail Alignment new_copy', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Rail Alignment new_copy', 1600, 900)
                cv2.imshow('Rail Alignment new_copy', vis_frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("   用户退出")
                    break
                elif key == ord('r'):
                    self.stop_motion()
                    self.frame_buffer.clear()
                    self.stable_lateral_error_mm = 0
                    self.last_pixel_offset_px = 0.0
                    self.last_rail_center_x = self.img_width / 2.0
                    self.last_target_center_x = self.pixel_lateral_calculator.target_center_x
                    self.last_rail_width_px = 0.0
                    self.last_step_mm = 0.0
                    self.last_stage_name = 'INIT'
                    self.last_motion_text = 'WAITING'
                    self.last_extract_reason = 'RESET'
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
                    with self.workflow_lock:
                        if not self.workflow_busy:
                            self.workflow_completed_cycles = 0
                            self.workflow_finished = False
                            self.workflow_failed = False
                            self.workflow_status_text = "ALIGNING"
                    with self.state_lock:
                        self.current_state = SystemState.IDLE
                    rospy.loginfo("   系统已重置")
                elif key == ord('p'):
                    paused = not paused
                    if paused:
                        self.stop_motion()
                        rospy.loginfo("⏸️  系统暂停")
                    else:
                        rospy.loginfo("▶️  系统继续")

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

# ========== 入口函数 ==========
def main():
    yolo_weights = os.environ.get(
        'RAIL_YOLO_WEIGHTS',
        '/home/ubuntu/mmLaneDet-master/work_dirs/yolov8s_327_from_3252_e80/weights/best.pt',
    )
    requested_device = os.environ.get('RAIL_YOLO_DEVICE', 'auto').strip().lower()
    if requested_device in {'', 'auto'}:
        device = '0' if torch.cuda.is_available() else 'cpu'
    elif requested_device != 'cpu' and not torch.cuda.is_available():
        rospy.logwarn(f"请求设备 {requested_device} 但当前无 CUDA, 自动切换到 CPU")
        device = 'cpu'
    else:
        device = requested_device

    yolo_imgsz = int(os.environ.get('RAIL_YOLO_IMGSZ', '960'))
    yolo_conf = float(os.environ.get('RAIL_YOLO_CONF', '0.20'))
    yolo_min_y = int(os.environ.get('RAIL_YOLO_MIN_Y', '530'))
    yolo_row_step = int(os.environ.get('RAIL_YOLO_ROW_STEP', '10'))

    try:
        controller = FusionController(
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

if __name__ == '__main__':
    main()
