# Description: This file contains the emergency_stop task.
import smach
from fsm_waypoint.states.subscriber import GetA012Emergency, GetA012Release, GetBumperEmergency, GetBumperRelease, CheckAccuracy
from fsm_waypoint.states.publisher import SoundPlay, SendZeroTwistByAccuracy, SendZeroTwist
from fsm_waypoint.utils import child_termination_cb, outcome_cb

def emergency_stop(node):
    sw_emergency = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                                input_keys=['blackboard'],
                                                output_keys=['blackboard'])

    with sw_emergency:
        smach.StateMachine.add('GET_BUMPER_EMERGENCY', GetBumperEmergency(node),
                               transitions={'bumper': 'BUMPER_PA_PRESSED', 'succeeded': 'BUMPER_EMERGENCY_BUTTON_PRESSED', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_EMERGENCY_BUTTON_PRESSED', SoundPlay(node, 'bumper_emergency_pressed'),
                               transitions={'succeeded': 'GET_BUMPER_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_BUMPER_RELEASE', GetBumperRelease(node),
                               transitions={'bumper': 'BUMPER_PA_PRESSED', 'succeeded': 'BUMPER_BUTTON_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_BUTTON_RELEASE', SoundPlay(node, 'bumper_emergency_release'),
                               transitions={'succeeded': 'GET_BUMPER_EMERGENCY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_PA_PRESSED', SoundPlay(node, 'bumper_pa_pressed'),
                               transitions={'succeeded': 'GET_BUMPER_EMERGENCY', 'preempted': 'preempted', # sound -> start
                                            'aborted': 'aborted'})
        # pa release은 없음
        # smach.StateMachine.add('BUMPER_PA_RELEASE', SoundPlay(node, 'bumper_pa_release'),
        #                        transitions={'succeeded': 'GET_BUMPER_EMERGENCY', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})


    hw_emergency = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=['blackboard'],
                                      output_keys=['blackboard'])

    with hw_emergency:
        smach.StateMachine.add('GET_A012_EMERGENCY', GetA012Emergency(node),
                               transitions={'succeeded': 'BUMPER_EMERGENCY_PRESSED', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_EMERGENCY_PRESSED', SoundPlay(node, 'bumper_emergency_pressed'),
                               transitions={'succeeded': 'GET_A012_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_A012_RELEASE', GetA012Release(node),
                               transitions={'succeeded': 'BUMPER_EMERGENCY_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_EMERGENCY_RELEASE', SoundPlay(node, 'bumper_emergency_release'),
                               transitions={'succeeded': 'GET_A012_EMERGENCY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})

    gps_emergency = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=['blackboard'],
                                      output_keys=['blackboard'])

    with gps_emergency:
        smach.StateMachine.add('CHECK_ACCURACY', CheckAccuracy(node), # safe delay
                               transitions={'succeeded': 'MONITOR_ACCURACY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MONITOR_ACCURACY', SendZeroTwistByAccuracy(node),  # until low accuracy
                               transitions={'succeeded': 'LOW_ACCURACY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('LOW_ACCURACY', SoundPlay(node, 'low_accuracy'),
                               transitions={'succeeded': 'SEND_ZERO_TWIST', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SEND_ZERO_TWIST', SendZeroTwist(node, 10),  # until high accuracy
                               transitions={'succeeded': 'CHECK_ACCURACY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})

    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('SW_EMERGENCY', sw_emergency)
        smach.Concurrence.add('HW_EMERGENCY', hw_emergency)
        smach.Concurrence.add('GPS_EMERGENCY', gps_emergency)
    return sm