#!/usr/bin/env python

import struct
import sys

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import smach
import smach_ros

import baxter_interface
from baxter_interface import CHECK_VERSION

class PrintIKResponse(smach.State):
    def __init__(self, input_keys):
        smach.State.__init__(self, input_keys=input_keys, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        print('userdata.joint_response: {}'.format(userdata.joint_response))
        print('userdata.RESULT_INVALID: {}'.format(userdata.RESULT_INVALID))
        print('userdata.result_type_response: {}'.format(userdata.result_type_response))

        resp_seeds = struct.unpack('<%dB' % len(userdata.result_type_response), userdata.result_type_response)

        print('resp_seeds: {}'.format(resp_seeds))

        if any(response == userdata.RESULT_INVALID for response in resp_seeds):
            return 'aborted'
        else:
            return 'succeeded'

def main():
    print("Starting Baxter SMACH Inverse Kinematics Service Test.")
    
    print("Initializing node...")
    rospy.init_node('baxter_smach_ik_test')
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
    
        sm.userdata.ik_request_poses = [PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='base'),
                                                    pose=Pose(position=Point(x=0.657579481614,
                                                                             y=0.851981417433,
                                                                             z=0.0388352386502),
                                                    orientation=Quaternion(x=-0.366894936773,
                                                                           y=0.885980397775,
                                                                           z=0.108155782462,
                                                                           w=0.262162481772)))]

        smach.StateMachine.add('LEFT_LIMB_IK_POSE_TO_JOINT_POSITIONS',
                               smach_ros.ServiceState('/ExternalTools/left/PositionKinematicsNode/IKService',
                                                      SolvePositionIK,
                                                      request_slots = ['pose_stamp'],
                                                      response_slots = ['joints', 'RESULT_INVALID', 'result_type']),
                               remapping={'pose_stamp':'ik_request_poses',
                                          'joints':'joint_response',
                                          'result_type':'result_type_response'},
                               transitions={'succeeded':'PRINT_IK_RESPONSE'})

        smach.StateMachine.add('PRINT_IK_RESPONSE',
                               PrintIKResponse(['joint_response', 'RESULT_INVALID', 'result_type_response']),
                               transitions={'succeeded':'succeeded', 'aborted':'aborted'})

    
    sis = smach_ros.IntrospectionServer('BAXTER_SMACH_IK_TEST_SERVER', sm, '/SM_ROOT')

    sis.start()

    outcome = sm.execute()
    
    print("Baxter SMACH Inverse Kinematics Service Test Complete. Ctrl-C to exit.")
    
    rospy.spin()

if __name__ == "__main__":
    main()
