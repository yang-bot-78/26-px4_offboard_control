#!/usr/bin/env python3
"""校验 vision_speed 携带的是经 child->world 旋转后的 FAST-LIO twist。

按 REP-147，FAST-LIO 的 twist 表达在 ``child_frame_id`` 下，而 MAVROS 的
``vision_speed`` 插件把这个向量当作局部世界 ENU 来解释，所以桥接必须做旋转。
本校验器用数值证明这次旋转，而不是靠肉眼看符号：对两个话题都存在的每一个时间戳，
用**同一条** odometry 样本里携带的姿态重算 ``R(child->world) * twist_child``，
再和桥接实际发布的值比对。

两个彼此独立的判定，因为单独任何一个都可能被骗过：

``ROTATION_MATCH``
    发布的向量等于重算出来的向量。这是真正的检查项。
``NOT_IDENTITY``
    发布的向量不等于未旋转的机体值。没有这一项的话，一场近水平、yaw 为零时录的
    数据会仅仅因为那里的 R 恰好是单位矩阵而"通过"第一项 —— 旋转完全缺失也看不出来。
    因此 R 接近单位矩阵时报 INCONCLUSIVE，而不是 PASS。

在线模式走订阅，bag 模式回放已录场次。两种模式都是只读：不发布话题、不调服务、
不写参数。
"""

import argparse
import math
import sys


def quat_to_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("四元数模长为零")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def rotate(matrix, vector):
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def key_of(stamp):
    return (stamp.sec, stamp.nanosec)


class Comparison:
    """累计逐样本结果并给出最终判定。"""

    def __init__(self, tolerance):
        self.tolerance = tolerance
        self.paired = 0
        self.rotated = 0
        self.identical = 0
        self.worst_error = 0.0
        self.max_r_deviation = 0.0
        self.unmatched_speed = 0

    def add(self, twist_child, orientation, published):
        matrix = quat_to_matrix(
            orientation.x, orientation.y, orientation.z, orientation.w)
        child = [twist_child.x, twist_child.y, twist_child.z]
        if not all(math.isfinite(v) for v in child):
            # 非有限值是桥接自己的拒绝路径，不是坐标系错误：直接跳过，
            # 不要算成一次不匹配。
            return
        expected = rotate(matrix, child)
        got = [published.x, published.y, published.z]
        error = max(abs(expected[i] - got[i]) for i in range(3))
        self.worst_error = max(self.worst_error, error)
        self.paired += 1
        if error <= self.tolerance:
            self.rotated += 1
        if max(abs(child[i] - got[i]) for i in range(3)) <= 0.0:
            self.identical += 1
        self.max_r_deviation = max(self.max_r_deviation, max(
            abs(matrix[i][j] - (1.0 if i == j else 0.0))
            for i in range(3) for j in range(3)))

    def report(self):
        if self.paired == 0:
            print("FAIL：没有任何时间戳配对成功的样本对。")
            print("  要么 vision_speed 没有发布者（检查桥接的 publish_speed 参数），")
            print("  要么两个话题的时间区间根本没有重叠。")
            return 2

        print(f"时间戳配对样本数              : {self.paired}")
        if self.unmatched_speed:
            print(f"未配对的 vision_speed 样本    : {self.unmatched_speed}")
        print(f"ROTATION_MATCH               : {self.rotated}/{self.paired}"
              f"  (最大偏差 {self.worst_error:.3e})")
        print(f"与未旋转 body 值完全相同      : {self.identical}/{self.paired}")
        print(f"max |R - I| 元素              : {self.max_r_deviation:.4f}")

        if self.rotated != self.paired:
            print()
            print("FAIL：发布的速度不等于 R(child->world)*twist。")
            print("  桥接没有施加本场次预期的旋转。")
            return 2

        # 防住模块开头说明的"近水平且 yaw≈0 时的假通过"。0.05 大约对应 3° 倾角：
        # 低于这个量，两个坐标系太接近，比较结果无法区分二者。
        if self.max_r_deviation < 0.05:
            print()
            print("INCONCLUSIVE：旋转在数值上完全精确，但整场 R 与单位矩阵的差")
            print(f"  始终在 {self.max_r_deviation:.4f} 以内，因此这个结果无法区分")
            print("  “旋转正确”和“旋转缺失”。")
            print("  请在真实姿态变化的区间上重跑 —— 无桨验证的 YAW90_FORWARD")
            print("  阶段就是为这一项设计的。")
            return 1

        print()
        print("PASS：已在非单位姿态上验证旋转。")
        return 0


def run_live(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           qos_profile_sensor_data)
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistWithCovarianceStamped
    import time

    comparison = Comparison(args.tolerance)
    odom = {}
    speeds = []

    rclpy.init()
    node = Node("check_twist_rotation")
    reliable = QoSProfile(depth=400, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

    def on_odom(msg):
        odom[key_of(msg.header.stamp)] = (
            msg.twist.twist.linear, msg.pose.pose.orientation)

    def on_speed(msg):
        speeds.append((key_of(msg.header.stamp), msg.twist.twist.linear))

    # 两种 QoS 都订阅一遍：里程计链路是 reliable 的，但如果上游是 best-effort
    # 发布者，只订 reliable 会静默地匹配不到任何东西。
    for qos in (reliable, qos_profile_sensor_data):
        node.create_subscription(Odometry, args.odom_topic, on_odom, qos)
        node.create_subscription(
            TwistWithCovarianceStamped, args.speed_topic, on_speed, qos)

    print(f"采样 {args.seconds:.0f} 秒：{args.odom_topic} 对比 {args.speed_topic}")
    deadline = time.time() + args.seconds
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    print(f"已收到 odometry {len(odom)} 条，vision_speed {len(speeds)} 条")
    seen = set()
    for stamp, linear in speeds:
        if stamp in seen:
            continue
        seen.add(stamp)
        if stamp not in odom:
            comparison.unmatched_speed += 1
            continue
        twist_child, orientation = odom[stamp]
        comparison.add(twist_child, orientation, linear)
    return comparison.report()


def run_bag(args):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id=""),
                ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (args.odom_topic, args.speed_topic):
        if topic not in types:
            print(f"FAIL：bag 中没有 {topic}")
            return 2

    comparison = Comparison(args.tolerance)
    odom = {}
    speeds = []
    classes = {name: get_message(kind) for name, kind in types.items()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == args.odom_topic:
            msg = deserialize_message(data, classes[topic])
            odom[key_of(msg.header.stamp)] = (
                msg.twist.twist.linear, msg.pose.pose.orientation)
        elif topic == args.speed_topic:
            msg = deserialize_message(data, classes[topic])
            speeds.append((key_of(msg.header.stamp), msg.twist.twist.linear))

    print(f"Bag：odometry {len(odom)} 条，vision_speed {len(speeds)} 条")
    for stamp, linear in speeds:
        if stamp not in odom:
            comparison.unmatched_speed += 1
            continue
        twist_child, orientation = odom[stamp]
        comparison.add(twist_child, orientation, linear)
    return comparison.report()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", help="分析已录制的 bag，而不是在线话题")
    parser.add_argument("--seconds", type=float, default=30.0,
                        help="在线采样窗口秒数（默认 30）")
    parser.add_argument("--odom-topic", default="/Odometry/healthy")
    parser.add_argument("--speed-topic",
                        default="/mavros/vision_speed/speed_twist_cov")
    parser.add_argument("--tolerance", type=float, default=1e-9,
                        help="逐轴匹配容差，单位 m/s（默认 1e-9）")
    args = parser.parse_args()
    return run_bag(args) if args.bag else run_live(args)


if __name__ == "__main__":
    sys.exit(main())
