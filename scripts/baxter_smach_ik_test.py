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

class PoseToJointTrajServiceState(smach.State):
    def __init__(self, ik_service_proxy, timeout=5.0, input_keys=['poses'], output_keys=['joints']):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys, output_keys=output_keys)
        self._ik_service_proxy = ik_service_proxy
        self._timeout = timeout

    def execute(self, userdata):
        # Set up a request object
        ik_request = SolvePositionIKRequest()

        # Parse poses from userdata and append to request
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        for pose in userdata.poses:
            if isinstance(pose, PoseStamped):
                ik_request.pose_stamp.append(pose)
            elif isinstance(pose, Pose):
                pose_stamped = PoseStamped(header=header, pose=pose)
                ik_request.pose_stamp.append(pose_stamped)
            elif isinstance(pose, list):
                position = Point(x=pose[0][0], y=pose[0][1], z=pose[0][2])
                orientation = Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3])
                pose_stamped = PoseStamped(header=header, pose = Pose(position=position, orientation=orientation))
                ik_request.pose_stamp.append(pose_stamped)
            else:
                return 'aborted'

        # Wait for service (important!)
        self._ik_service_proxy.wait_for_service(self._timeout)

        # Receive response
        ik_response = self._ik_service_proxy(ik_request)

        # Check response validity and return result appropriately
        resp_seeds = struct.unpack('<%dB' % len(ik_response.result_type), ik_response.result_type)
        if any(response == ik_response.RESULT_INVALID for response in resp_seeds):
            return 'aborted'
        else:
            userdata.joints = ik_response.joints
            return 'succeeded'


class PrintUserdataState(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['input'], outcomes=['succeeded'])

    def execute(self, userdata):
        print('type(userdata.input): {}'.format(type(userdata.input)))
        if isinstance(userdata.input, list):
            print('type(userdata.input[0]): {}'.format(type(userdata.input[0])))
        print('userdata.input: {}'.format(userdata.input))

        return 'succeeded'

def main():
    print("Starting Baxter SMACH Inverse Kinematics Service Test.")
    
    print("Initializing node...")
    rospy.init_node('baxter_smach_ik_test')
        
    print("Initializing inverse kinematics service proxies for each limb... ")
    left_limb_ik_service_proxy = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
    right_limb_ik_service_proxy = rospy.ServiceProxy('/ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
    
        sm.userdata.ik_request_poses = [[[0.657579481614, 0.851981417433, 0.0388352386502],
                                         [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]]]
        
        smach.StateMachine.add('LEFT_LIMB_IK_POSE_TO_JOINT_POSITIONS',
                               PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                               remapping={'poses':'ik_request_poses', 'joints':'joint_response'},
                               transitions={'succeeded':'PRINT_IK_RESPONSE'})

        smach.StateMachine.add('PRINT_IK_RESPONSE',
                               PrintUserdataState(),
                               remapping={'input': 'joint_response'},
                               transitions={'succeeded':'succeeded'})

    
    sis = smach_ros.IntrospectionServer('BAXTER_SMACH_IK_TEST_SERVER', sm, '/SM_ROOT')

    sis.start()

    outcome = sm.execute()
    
    print("Baxter SMACH Inverse Kinematics Service Test Complete. Ctrl-C to exit.")
    
    rospy.spin()

if __name__ == "__main__":
    main()
