# ros2
import rclpy
from geometry_msgs.msg import Twist

# fsm
import smach
# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class SendZeroTwist(smach.State):
    def __init__(self, node, timeout):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.timeout = timeout
        self.node = node
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'teleop_node/cmd_vel', 10)

    def execute(self, ud):
        start_time = time.time()
        outcome = 'aborted'
        while True:
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break

            # Send zero twist message periodically
            self.send_zero_twist()

            time.sleep(0.1)
            if time.time() - start_time > self.timeout:
                outcome = 'succeeded'
                break

        return outcome

    def send_zero_twist(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        self.node.get_logger().info("Published zero twist due to trigger.")
