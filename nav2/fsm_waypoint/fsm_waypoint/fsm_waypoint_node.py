# Description: FSM for waypoint navigation
# ros2
import rclpy
from rclpy.node import Node

# fsm
import smach
# utils
from fsm_waypoint.utils import debug, info, warning, error, critical
from fsm_waypoint.utils import init_blackboard

# scenario
from fsm_waypoint.scenario import bringup
from fsm_waypoint.scenario import ready
from fsm_waypoint.scenario import bearing
from fsm_waypoint.scenario import waypoint
from fsm_waypoint.scenario import path
from fsm_waypoint.scenario import create
from fsm_waypoint.scenario import update
from fsm_waypoint.scenario import obstacle
from fsm_waypoint.scenario import rewaypoint
from fsm_waypoint.scenario import repath
from fsm_waypoint.scenario import rmf


# Create the state machine
def main(args=None):
    rclpy.init(args=args)
    node = Node('fsm_waypoint_node')
    node.declare_parameter('test01', 100.0)
    node.declare_parameter('test02', False)
    smach.set_loggers(info, warning, debug, error)
    # Create state machine
    top = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    top.userdata.blackboard = init_blackboard()

    with top:
        smach.StateMachine.add('BRINGUP', bringup(node),
                               transitions={'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('READY', ready(node),
                               transitions={'bearing': 'BEARING', 'waypoint': 'WAYPOINT', 'path': 'PATH', 'rmf': 'RMF',
                                            'create': 'CREATE', 'update': 'UPDATE', 'cancel': 'READY',
                                            'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('BEARING', bearing(node),
                               transitions={'cancel': 'READY',
                                   'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

        # # todo: obstacle or stop_and_go : waypoint type 확인후 결정
        smach.StateMachine.add('WAYPOINT', waypoint(node),
                               transitions={'cancel': 'READY', 'navfn': 'OBSTACLE_WAYPOINT',
                                   'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})
        smach.StateMachine.add('REWAYPOINT', rewaypoint(node),
                               transitions={'cancel': 'READY', 'navfn': 'OBSTACLE_WAYPOINT',
                                            'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('PATH', path(node),
                               transitions={'cancel': 'READY', 'navfn': 'OBSTACLE_PATH',
                                   'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})
        smach.StateMachine.add('REPATH', repath(node),
                               transitions={'cancel': 'READY', 'navfn': 'OBSTACLE_PATH',
                                            'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('CREATE', create(node),
                               transitions={'cancel': 'READY',
                                   'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('UPDATE', update(node),
                               transitions={'cancel': 'READY',
                                   'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('OBSTACLE_WAYPOINT', obstacle(node),
                               transitions={'cancel': 'READY',
                                   'succeeded': 'REWAYPOINT', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('OBSTACLE_PATH', obstacle(node),
                               transitions={'cancel': 'READY',
                                            'succeeded': 'REPATH', 'preempted': 'preempted', 'aborted': 'aborted'})

        smach.StateMachine.add('RMF', rmf(node),
                               transitions={'cancel': 'READY', 'succeeded': 'READY', 'preempted': 'preempted', 'aborted': 'aborted'})

    outcome = top.execute()
    if outcome == 'overall_success':
        info('State machine executed successfully')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
