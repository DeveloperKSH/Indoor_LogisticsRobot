# ros2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String

# qos
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class SendZeroTwistByAccuracy(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])

        self.node = node
        self.timeout = timeout
        self.sub_ = None
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            'emergency/cmd_vel',
            qos_profile=transient_qos)
        self.sound_pub = self.node.create_publisher(
            String, '/sound_file',
            qos_profile=transient_qos)

        self.horizontal_acc = None
        self.vertical_acc = None
        self.poor_accuracy_start = None
        self.good_accuracy_start = None
        self.poor_condition_met = False  # poor 상태 10초 조건 만족 여부

    def execute(self, ud):
        self.sub_ = self.node.create_subscription(DiagnosticArray,
                                                  'diagnostics',
                                                  self.diagnostics_callback,
                                                  10)

        # 상태 초기화
        self.horizontal_acc = None
        self.vertical_acc = None
        self.poor_accuracy_start = None
        self.good_accuracy_start = None
        self.poor_condition_met = False

        start_time = time.time()
        outcome = 'aborted'

        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break

            current_time = time.time()

            if self.horizontal_acc is not None and self.vertical_acc is not None:
                is_poor = self.horizontal_acc > 0.03 or self.vertical_acc > 0.04

                if is_poor:
                    # self.good_accuracy_start = None  # good 상태 초기화

                    if self.poor_accuracy_start is None:
                        self.poor_accuracy_start = current_time
                        debug("📉 Poor GPS accuracy 시작")

                    elif current_time - self.poor_accuracy_start >= 2:
                        if not self.poor_condition_met:
                            info("⚠️ GPS Horizontal 정확도가 0.03m과 Vertical 정확도가 0.04m를 초과한 상태가 2초 지속되었습니다.")
                            # self.publish_sound_file('/root/scripts/sound/sounds/voice/low_accuracy.mp3')
                            # self.poor_condition_met = True
                            outcome = 'succeeded'
                            break
                        # self.send_zero_twist()

                else:
                    self.poor_accuracy_start = None  # poor 상태 초기화
                #
                #     if self.poor_condition_met:
                #         if self.good_accuracy_start is None:
                #             self.good_accuracy_start = current_time
                #             debug("📈 GPS 정확도 회복, 타이머 시작")
                #
                #         elif current_time - self.good_accuracy_start >= 10:
                #             info("✅ GPS 정확도가 0.03m 이하로 회복된 지 10초 지났습니다. 재진행합니다.")
                #             self.publish_sound_file('/root/scripts/sound/sounds/voice/high_accuracy.mp3')
                #             outcome = 'succeeded'
                #             break
                #         else:
                #             self.send_zero_twist()  # 회복 후 10초 전까지 정지 유지
                #     else:
                #         # 아직 poor 상태가 10초 미만이었던 경우
                #         self.good_accuracy_start = None

            time.sleep(1)
            if self.timeout and (current_time - start_time > self.timeout):
                warning("⏳ Timeout reached while monitoring GPS accuracy.")
                outcome = 'succeeded'
                break

        self.node.destroy_subscription(self.sub_)
        return outcome

    def diagnostics_callback(self, msg):
        for status in msg.status:
            if status.name == "ublox_gps_node: fix":
                for value in status.values:
                    if value.key == "Horizontal Accuracy [m]":
                        self.horizontal_acc = float(value.value)
                    elif value.key == "Vertical Accuracy [m]":
                        self.vertical_acc = float(value.value)

    def send_zero_twist(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.node.get_logger().info("📢 Published zero twist.")

    def publish_sound_file(self, sound_file_path):
        msg = String()
        msg.data = sound_file_path
        self.sound_pub.publish(msg)
        info(f'Published sound file: {sound_file_path}')