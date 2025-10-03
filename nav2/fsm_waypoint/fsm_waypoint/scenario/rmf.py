
import smach
from fsm_waypoint.tasks import emergency_stop
from fsm_waypoint.tasks import network_error
from fsm_waypoint.tasks import spin_once
from fsm_waypoint.utils import child_termination_cb, outcome_cb
from fsm_waypoint.states.publisher import FleetManager
from fsm_waypoint.states.navigator import RMFNavToPose
from fsm_waypoint.tasks import wss_cancel_command

# YAML configuration as a Python dictionary
fleet_config = {
    'rmf_fleet': {
        'name': "gl_fleet",
        'limits': {
            'linear': [0.5, 0.75],  # velocity, acceleration
            'angular': [0.6, 2.0]   # velocity, acceleration
        },
        'profile': {
            'footprint': 0.3,  # radius in m
            'vicinity': 0.5    # radius in m
        },
        'reversible': True,  # whether robots in this fleet can reverse
        'battery_system': {
            'voltage': 12.0,  # V
            'capacity': 24.0, # Ahr
            'charging_current': 5.0 # A
        },
        'mechanical_system': {
            'mass': 20.0,  # kg
            'moment_of_inertia': 10.0,  # kgm^2
            'friction_coefficient': 0.22
        },
        'ambient_system': {
            'power': 20.0  # W
        },
        'tool_system': {
            'power': 0.0  # W
        },
        'recharge_threshold': 0.10,  # Battery level below which robots in this fleet will not operate
        'recharge_soc': 1.0,  # Battery level to which robots in this fleet should be charged up to during recharging tasks
        'publish_fleet_state': 10.0,  # Publish frequency for fleet state
        'account_for_battery_drain': True,
        'task_capabilities': {
            'loop': True,
            'delivery': True
        },
        'finishing_request': "nothing",  # [park, charge, nothing]
        'responsive_wait': True,  # Should responsive wait be on/off for the whole fleet by default?
        'robots': {
            'tester': {
                'charger': "s1",
                'responsive_wait': False,
                'destination': ["d3-1, d3-2, d4-1, d4-2, d5-1, d5-2"]   # gl ["d1,d2"]
            }
            # Additional robots can be added here
        }
    },
    'fleet_manager': {
        'ip': "0.0.0.0",
        'port': 22011,
        'user': "some_user",
        'password': "some_password",
        'reference_coordinates': {
            'L1': {
                'rmf': [
                    [8.4909, -3.0222],
                    [10.5734, -4.7068],
                    [19.4283, -6.1121],
                    [7.8983, -6.2983]
                ],
                'robot': [
                    [0.6489, 2.5933],
                    [3.0463, 1.3754],
                    [12.0011, 1.7940],
                    [0.7717, -0.6532]
                ]
            }
        }
    }
}

def rmf(node):
    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'cancel'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('WSS_CANCEL_COMMAND', wss_cancel_command(node))
        smach.Concurrence.add('EMERGENCY_STOP', emergency_stop(node))
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))
        smach.Concurrence.add('NETWORK_ERROR', network_error(node))
        smach.Concurrence.add('FLEET_MANAGER', FleetManager(node, fleet_config))
        smach.Concurrence.add('RMF_GO_TO_POSE', RMFNavToPose(node, fleet_config))
    return sm