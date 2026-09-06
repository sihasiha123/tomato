#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试用: 手动发布导航成功状态，放开 traverse 对轨门控。

正式跑导航时不要启动本脚本；只有单独测试 new_copy_no_capture + traverse 时使用。
"""

import argparse
import math
import os
import sys

import rospy


DE5_PYTHON_PATH = "/home/ubuntu/Desktop/de5/robot/putn_ws/devel/lib/python3/dist-packages"
if DE5_PYTHON_PATH not in sys.path and os.path.isdir(DE5_PYTHON_PATH):
    sys.path.insert(0, DE5_PYTHON_PATH)

from rover_msgs.msg import roverGoalStatus  # noqa: E402


def yaw_to_quaternion(yaw_deg):
    yaw_rad = math.radians(float(yaw_deg))
    half = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half), math.cos(half)


def build_status(args):
    msg = roverGoalStatus()
    msg.x = float(args.x)
    msg.y = float(args.y)
    msg.z = float(args.z)
    qx, qy, qz, qw = yaw_to_quaternion(args.yaw_deg)
    msg.orientation_x = qx
    msg.orientation_y = qy
    msg.orientation_z = qz
    msg.orientation_w = qw
    msg.status = roverGoalStatus.SUCCEEDED
    msg.goal_id = int(args.goal_id)
    msg.text = args.text
    return msg


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish fake /cur_global_goal_status SUCCEEDED for traverse testing."
    )
    parser.add_argument("--topic", default="/cur_global_goal_status")
    parser.add_argument("--rate", type=float, default=1.0, help="publish rate in Hz")
    parser.add_argument("--once", action="store_true", help="publish a few times then exit")
    parser.add_argument("--repeat", type=int, default=8, help="repeat count when --once is used")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--goal-id", type=int, default=0)
    parser.add_argument("--text", default="fake navigation success for traverse test")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("fake_nav_success_publisher", anonymous=True)
    pub = rospy.Publisher(args.topic, roverGoalStatus, queue_size=4, latch=True)
    msg = build_status(args)
    rate_hz = max(0.1, float(args.rate))
    rate = rospy.Rate(rate_hz)

    rospy.loginfo(
        "测试发布导航成功: topic=%s, status=SUCCEEDED(%d), once=%s",
        args.topic,
        int(msg.status),
        bool(args.once),
    )

    if args.once:
        for _ in range(max(1, int(args.repeat))):
            if rospy.is_shutdown():
                break
            pub.publish(msg)
            rate.sleep()
        rospy.loginfo("测试导航成功发布完成，退出")
        return 0

    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo_throttle(
            3.0,
            "持续发布测试导航成功: topic=%s, subscribers=%d",
            args.topic,
            pub.get_num_connections(),
        )
        rate.sleep()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
