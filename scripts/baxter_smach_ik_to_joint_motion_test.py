#!/usr/bin/env python

import struct
import sys

from copy import copy

import rospy
import rostopic

import actionlib

import threading

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

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from sensor_msgs.msg import JointState

import smach
import smach_ros

import baxter_interface
from baxter_interface import CHECK_VERSION


class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


class WaitForMsgState(smach.State):
    """This class acts as a generic message listener with blocking, timeout, latch and flexible usage.

    It is meant to be extended with a case specific class that initializes this one appropriately
    and contains the msg_cb (or overrides execute if really needed).

    Its waitForMsg method implements the core functionality: waiting for the message, returning
    the message itself or None on timeout.

    Its execute method wraps the waitForMsg and returns succeeded or aborted, depending on the returned
    message beeing existent or None. Additionally, in the successfull case, the msg_cb, if given, will
    be called with the message and the userdata, so that a self defined method can convert message data to
    smach userdata.
    Those userdata fields have to be passed via 'output_keys'.

    If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
    If msg_cb returns True, execute() will return "succeeded".
    If msg_cb returns False, execute() will return "aborted".
    If msg_cb has no return statement, execute() will act as described above.

    If thats still not enough, execute() might be overridden.

    latch: If True waitForMsg will return the last received message, so one message might be returned indefinite times.
    timeout: Seconds to wait for a message, defaults to None, disabling timeout
    output_keys: Userdata keys that the message callback needs to write to.
    """

    def __init__(self, topic, msg_type, msg_cb=None, output_keys=None, latch=False, timeout=None):
        if output_keys is None:
            output_keys = []
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        """Await and return the message or None on timeout."""
        rospy.loginfo('Waiting for message...')
        if self.timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while self.timeout is None or rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg is not None:
                rospy.loginfo('Got message.')
                message = self.msg

                if not self.latch:
                    self.msg = None

                self.mutex.release()
                return message
            self.mutex.release()

            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('waitForMsg is preempted!')
                return 'preempted'

            rospy.sleep(.1) # TODO: maybe convert ROSInterruptException into valid outcome

        rospy.loginfo('Timeout on waiting for message!')
        return None

    def execute(self, ud):
        """Default simplest execute(), see class description."""
        msg = self.waitForMsg()
        if msg is not None:
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'aborted'
            return 'succeeded'
        else:
            return 'aborted'


# class ReadTopicState(WaitForMsgState):
#     def __init__(self, topic, msg_type, fields, **kwargs):
#         self._fields = fields
#         WaitForMsgState.__init__(self, topic, msg_type, msg_cb=self._msg_cb, latch=True, output_keys=fields, **kwargs)
# 
#     def _msg_cb(self, msg, userdata):
#         for field in self._fields:
#             field_reader = rostopic.msgevalgen(field)
#             setattr(userdata, field, field_reader(msg))
# 
#         return msg is not None


class ReadLimbJointsState(WaitForMsgState):
    """ This is a state for reading the current joint positions from a specified limb.
    """
    def __init__(self, limb, topic='/robot/joint_states', **kwargs):
        self._limb = limb
        
        # Use the super class to read from topic
        super(ReadLimbJointsState, self).__init__(topic, JointState, msg_cb=self._msg_cb, latch=True, output_keys=['joints'], **kwargs)

    def _msg_cb(self, msg, userdata):
        # Read 'name' and 'position' fields from topic message
        name_field_reader = rostopic.msgevalgen('name')
        joint_names = name_field_reader(msg)
        position_field_reader = rostopic.msgevalgen('position')
        joint_positions = position_field_reader(msg)
        
        # Create a dict of names/positions
        current_joint_positions = dict(zip(joint_names, joint_positions))

        # Create a userdata JointState entry using the joint positions for the selected limb
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        limb_joint_names = [self._limb + joint_name for joint_name in ['_s1', '_e0', '_e1', '_w0', '_w1', '_w2']]
        limb_joint_positions = [current_joint_positions[limb_joint_name] for limb_joint_name in limb_joint_names]
        userdata.joints = JointState(header=header, name=limb_joint_names, position=limb_joint_positions)

        return msg is not None


class FollowJointTrajActionState(smach.State):
    """ This is a state for executing a follow joint trajectory action over a
        series of via points using a Baxter Trajectory action client class.

        This class is necessary beyond the use of a SimpleActionState in
        combination with a FollowJointTrajectoryAction, because in order to use
        the Baxter interface succesfully, the trajectory SimpleActionClient must be
        instructed to wait for the result of the action using a call to wait_for_result(),
        something that is not possible to specify using a SimpleActionState.
        
        Inputs:
            Default userdata input keys are 'points' and 'times' specifying
            lists of via points and time durations for each part of the trajectory
            respectively.

            However, these may be overridden with other input keys and manipulated
            by callback functions specified by points_cb and times_cb respectively.
            This is useful in cases where existing userdata point lists need to be
            re-ordered, for example, or for when points specified in multiple userdata
            keys need to be combined, e.g. when adding the current joint positions as
            read from another state to a user-specified list.

            Note that, since these callbacks do not alter the SMACH interface of the state,
            we do not need to use the cb_interface decorator etc. in this context.
            
    """
    def __init__(self, traj_client, timeout=15.0, input_keys = ['points', 'times'], points_cb = None, times_cb = None):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys)

        # Save trajectory action client wait_for_result timeout
        self._timeout = timeout

        # Save reference to trajectory client object
        self._traj_client = traj_client
        rospy.on_shutdown(self._traj_client.stop)

        # Set up a points callback
        self._points_cb = points_cb
        
        # Set up a times callback
        self._times_cb = times_cb

    def execute(self, userdata):
        # If a points callback has been defined, use it to format
        # points specified by the input keys in the userdata
        print('I make it into execute()...')
        if self._points_cb:
            points = self._points_cb(userdata)
        else:
            points = userdata.points
        
        # If a times callback has been defined, use it to format
        # times specified by the input keys in the userdata
        if self._times_cb:
            times = self._times_cb(userdata)
        else:
            times = userdata.times

        # Add via points and times to trajectory
        for point, time in zip(points, userdata.times):
            if isinstance(point, JointState):
                point = point.position
            print('type(point): {}'.format(type(point)))
            print('point: {}'.format(point))
            self._traj_client.add_point(point, time)

        # Start motion
        print('I make it as far as start()...')
        self._traj_client.start()

        # Wait for result from action client (important!)
        print('Could this be where the problem lies?')
        self._traj_client.wait(self._timeout)

        return 'succeeded'

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
    
    print("Initializing follow joint trajectory action clients for each limb... ")
    left_traj_client = Trajectory('left')
    right_traj_client = Trajectory('right')
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        
        smach.StateMachine.add('READ_LEFT_LIMB_JOINTS_1',
                               ReadLimbJointsState('left'),
                               remapping={'joints':'left_limb_joint_positions_1'},
                               transitions={'succeeded':'LEFT_LIMB_GO_TO_START_POSITION'})
    
        sm.userdata.left_limb_joint_start_positions = (
                [0.6699952259595108,
                  1.030009435085784,
                  -0.4999997247485215,
                  -1.189968899785275,
                  1.9400238130755056,
                  -0.08000397926829805,
                  -0.9999781166910306] )
        
        sm.userdata.left_limb_start_traj_times = [0.0, 7.0]
        
        smach.StateMachine.add('LEFT_LIMB_GO_TO_START_POSITION',
                               FollowJointTrajActionState(left_traj_client,
                                                          input_keys = ['left_limb_joint_positions_1',
                                                                        'left_limb_joint_start_positions',
                                                                        'times'],
                                                          points_cb = lambda ud: [ud.left_limb_joint_positions_1] +
                                                                                 [ud.left_limb_joint_start_positions]),
                               remapping={'times':'left_limb_start_traj_times'},
                               transitions={'succeeded':'LEFT_LIMB_IK_POSE_TO_JOINT_POSITIONS'})
    
        # sm.userdata.ik_request_poses = [[[0.657579481614, 0.851981417433, 0.0388352386502],
        #                                  [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]]]

        sm.userdata.ik_request_poses = [[[0.7, 0.15, -0.129],
                                         [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]]]
        
        smach.StateMachine.add('LEFT_LIMB_IK_POSE_TO_JOINT_POSITIONS',
                               PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                               remapping={'poses':'ik_request_poses', 'joints':'left_limb_ik_joint_response_1'},
                               transitions={'succeeded':'LEFT_LIMB_IK_JOINT_MOTION'})
        
        sm.userdata.left_limb_ik_joint_traj_times = [0.0, 7.0]
        
        smach.StateMachine.add('LEFT_LIMB_IK_JOINT_MOTION',
                               FollowJointTrajActionState(left_traj_client,
                                                          input_keys = ['left_limb_joint_start_positions',
                                                                        'left_limb_ik_joint_response_1',
                                                                        'times'],
                                                          points_cb = lambda ud: [ud.left_limb_joint_start_positions] +
                                                                                 [ud.left_limb_ik_joint_response_1]),
                               remapping={'times':'left_limb_ik_joint_traj_times'},
                               transitions={'succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('BAXTER_SMACH_IK_TEST_SERVER', sm, '/SM_ROOT')

    sis.start()

    outcome = sm.execute()
    
    print("Baxter SMACH Inverse Kinematics Service Test Complete. Ctrl-C to exit.")
    
    rospy.spin()

if __name__ == "__main__":
    main()
