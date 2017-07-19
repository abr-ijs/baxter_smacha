#!/usr/bin/env python

import struct
import sys
import os

from copy import copy

import rospy
import rostopic
import rospkg

import actionlib

import threading

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

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
        limb_joint_names = [self._limb + joint_name for joint_name in ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']]
        limb_joint_positions = [current_joint_positions[limb_joint_name] for limb_joint_name in limb_joint_names]
        userdata.joints = JointState(header=header, name=limb_joint_names, position=limb_joint_positions)

        return msg is not None


class MoveToJointPositionState(smach.State):
    """ This state moves a given limb to a specified joint position using the Baxter inteface.

        Note that this state does not make use of the joint trajectory action server.
    """
    def __init__(self, limb_interface, timeout=15.0, input_keys = ['positions']):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys)
        self._limb_interface = limb_interface
        self._timeout = timeout

    def execute(self, userdata):
        if userdata.positions:
            if isinstance(userdata.positions, list):
                limb_joint_names = [self._limb_interface.name + joint_name for joint_name in ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']]
                positions = dict(zip(limb_joint_names, userdata.positions))
            elif isinstance(userdata.positions, JointState):
                positions = dict(zip(userdata.positions.name, userdata.positions.position))
            else:
                positions = userdata.positions

            self._limb_interface.move_to_joint_positions(positions, timeout=self._timeout)
        else:
            return 'aborted'

        return 'succeeded'


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
            self._traj_client.add_point(point, time)

        # Start motion
        self._traj_client.start()

        # Wait for result from action client (important!)
        self._traj_client.wait(self._timeout)

        return 'succeeded'

class PoseToJointTrajServiceState(smach.State):
    def __init__(self, ik_service_proxy, timeout=5.0, input_keys=['poses'], output_keys=['joints'], poses_cb = None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys, output_keys=output_keys)
        self._ik_service_proxy = ik_service_proxy
        self._timeout = timeout
        
        # Set up a poses callback
        self._poses_cb = poses_cb

    def execute(self, userdata):
        # If a poses callback has been defined, use it to format
        # poses specified by the input keys in the userdata
        if self._poses_cb:
            poses = self._poses_cb(userdata)
        else:
            poses = userdata.poses

        print('poses: {}'.format(poses))

        # Set up a request object
        ik_request = SolvePositionIKRequest()

        # Parse poses from userdata and append to request
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        for pose in poses:
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
        try:
            ik_response = self._ik_service_proxy(ik_request)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 'aborted'

        # Check response validity and return result appropriately
        resp_seeds = struct.unpack('<%dB' % len(ik_response.result_type), ik_response.result_type)
        if any(response == ik_response.RESULT_INVALID for response in resp_seeds):
            print('resp_seeds: {}'.format(resp_seeds))
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


class LoadGazeboModelState(smach.State):
    def __init__(self, input_keys=['name', 'model_path', 'pose', 'reference_frame'], pose_cb = None):
        smach.State.__init__(self, input_keys=input_keys, outcomes=['succeeded'])
        
        # Set up a poses callback
        self._pose_cb = pose_cb

    def execute(self, userdata):
        # If a pose callback has been defined, use it to format
        # pose specified by the input keys in the userdata
        if self._pose_cb:
            pose = self._pose_cb(userdata)
        else:
            pose = userdata.pose

        # Parse pose
        if isinstance(pose, PoseStamped):
            pose = pose.pose
        elif isinstance(pose, Pose):
            pose = pose
        elif isinstance(pose, list):
            position = Point(x=pose[0][0], y=pose[0][1], z=pose[0][2])
            orientation = Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3])
            pose = Pose(position=position, orientation=orientation)
        else:
            return 'aborted'

        # Load model SDF/URDF XML
        model_xml = ''
        with open(userdata.model_path, 'r') as model_file:
            model_xml = model_file.read().replace('\n', '')

        # Spawn model SDF/URDF
        if os.path.splitext(userdata.model_path)[1][1:].lower() == 'sdf':
            spawn_service_type = 'sdf'
        elif os.path.splitext(userdata.model_path)[1][1:].lower() == 'urdf':
            spawn_service_type = 'urdf'

        try:
            spawn_service_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_response = spawn_service_proxy(userdata.name, model_xml, "/",
                                                 pose, userdata.reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr('Spawn ' + spawn_service_type.upper() + ' service call failed: {0}'.format(e))

        return 'succeeded'

def delete_gazebo_model(model):
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model(model)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

class GripperInterfaceState(smach.State):
    def __init__(self, gripper_interface, command):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self._gripper_interface = gripper_interface
        self._command = command

    def execute(self, userdata):
        if self._command == 'open':
            self._gripper_interface.open()
            rospy.sleep(1.0)
        elif self._command == 'close':
            self._gripper_interface.close()
            rospy.sleep(1.0)
        else:
            return 'aborted'

        return 'succeeded'




def main():
    print("Starting Baxter SMACH Inverse Kinematics Service Test.")
    
    print("Initializing node...")
    rospy.init_node('baxter_smach_ik_test')
    
    print("Initializing interfaces for each limb... ")
    left_limb_interface = baxter_interface.Limb('left')
    right_limb_interface = baxter_interface.Limb('right')
    
    print("Initializing interfaces for each gripper... ")
    left_gripper_interface = baxter_interface.Gripper('left')
    right_gripper_interface = baxter_interface.Gripper('right')
        
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
    
        sm.userdata.overhead_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]
        sm.userdata.hover_offset = [0.0, 0.0, 0.15]

        sm.userdata.table_model_name = 'cafe_table'
        sm.userdata.table_model_path = rospkg.RosPack().get_path('baxter_sim_examples')+'/models/cafe_table/model.sdf'
        sm.userdata.table_model_pose_world = Pose(position=Point(x=1.0, y=0.0, z=0.0))
        sm.userdata.table_model_ref_frame = 'world'

        smach.StateMachine.add('LOAD_TABLE_MODEL',
                               LoadGazeboModelState(),
                               remapping={'name':'table_model_name',
                                          'model_path':'table_model_path',
                                          'pose':'table_model_pose_world',
                                          'reference_frame':'table_model_ref_frame'},
                               transitions={'succeeded':'LOAD_BLOCK_MODEL'})
        
        sm.userdata.block_model_name = 'block'
        sm.userdata.block_model_path = rospkg.RosPack().get_path('baxter_sim_examples')+'/models/block/model.urdf'
        # sm.userdata.block_model_pose = Pose(position=Point(x=0.6725, y=0.1265, z=0.7825))
        sm.userdata.block_model_pick_pose_world = [[0.6725, 0.1265, 0.7825], [0.0, 0.0, 0.0, 0.0]]
        sm.userdata.block_model_pick_ref_frame = 'world'
        sm.userdata.block_model_pick_pose = [[0.7, 0.15, -0.129], [0.0, 0.0, 0.0, 0.0]]
        
        smach.StateMachine.add('LOAD_BLOCK_MODEL',
                               LoadGazeboModelState(),
                               remapping={'name':'block_model_name',
                                          'model_path':'block_model_path',
                                          'pose':'block_model_pick_pose_world',
                                          'reference_frame':'block_model_pick_ref_frame'},
                               transitions={'succeeded':'READ_LEFT_LIMB_JOINTS_1'})
        
        smach.StateMachine.add('READ_LEFT_LIMB_JOINTS_1',
                               ReadLimbJointsState('left'),
                               remapping={'joints':'left_limb_joint_positions_1'},
                               transitions={'succeeded':'PRINT_START_JOINT_POSITIONS'})
        
        smach.StateMachine.add('PRINT_START_JOINT_POSITIONS',
                               PrintUserdataState(),
                               remapping={'input': 'left_limb_joint_positions_1'},
                               transitions={'succeeded':'LEFT_LIMB_MOVE_TO_START_POSITION'})
    
        sm.userdata.left_limb_joint_start_positions = (
                [-0.08000397926829805,
                 -0.9999781166910306,
                 -1.189968899785275,
                  1.9400238130755056,
                  0.6699952259595108,
                  1.030009435085784,
                 -0.4999997247485215] )
        
        smach.StateMachine.add('LEFT_LIMB_MOVE_TO_START_POSITION',
                               MoveToJointPositionState(left_limb_interface),
                               remapping={'positions':'left_limb_joint_start_positions'},
                               transitions={'succeeded':'LEFT_LIMB_IK_PICK_OBJECT_HOVER_POSE'})
    
        # sm.userdata.ik_request_position = [0.7, 0.15, -0.129]

        smach.StateMachine.add('LEFT_LIMB_IK_PICK_OBJECT_HOVER_POSE',
                               PoseToJointTrajServiceState(left_limb_ik_service_proxy,
                                                           input_keys = ['block_model_pick_pose',
                                                                         'overhead_orientation',
                                                                         'hover_offset'],
                                                           poses_cb = lambda ud: [[[a + b for a, b in zip(ud.block_model_pick_pose[0], ud.hover_offset)], ud.overhead_orientation]]),
                               remapping={'joints':'left_limb_ik_joint_response_1'},
                               transitions={'succeeded':'LEFT_LIMB_MOVE_TO_PICK_OBJECT_HOVER_POSE'})
        
        sm.userdata.left_limb_move_to_pick_object_hover_pose_traj_times = [0.0, 12.0]
        
        smach.StateMachine.add('LEFT_LIMB_MOVE_TO_PICK_OBJECT_HOVER_POSE',
                               FollowJointTrajActionState(left_traj_client,
                                                          input_keys = ['left_limb_joint_start_positions',
                                                                        'left_limb_ik_joint_response_1',
                                                                        'times'],
                                                          points_cb = lambda ud: [ud.left_limb_joint_start_positions] +
                                                                                 ud.left_limb_ik_joint_response_1),
                               remapping={'times':'left_limb_move_to_pick_object_hover_pose_traj_times'},
                               transitions={'succeeded':'LEFT_LIMB_OPEN_GRIPPER'})

        smach.StateMachine.add('LEFT_LIMB_OPEN_GRIPPER',
                               GripperInterfaceState(left_gripper_interface, 'open'),
                               transitions={'succeeded':'LEFT_LIMB_IK_PICK_OBJECT_GRIP_POSE'})
        
        smach.StateMachine.add('LEFT_LIMB_IK_PICK_OBJECT_GRIP_POSE',
                               PoseToJointTrajServiceState(left_limb_ik_service_proxy,
                                                           input_keys = ['block_model_pick_pose',
                                                                         'overhead_orientation'],
                                                           poses_cb = lambda ud: [[ud.block_model_pick_pose[0], ud.overhead_orientation]]),
                               remapping={'joints':'left_limb_ik_joint_response_2'},
                               transitions={'succeeded':'LEFT_LIMB_MOVE_TO_PICK_OBJECT_GRIP_POSE'})
        
        sm.userdata.left_limb_move_to_pick_object_grip_pose_traj_times = [0.0, 12.0]
        
        smach.StateMachine.add('LEFT_LIMB_MOVE_TO_PICK_OBJECT_GRIP_POSE',
                               FollowJointTrajActionState(left_traj_client,
                                                          input_keys = ['left_limb_ik_joint_response_1',
                                                                        'left_limb_ik_joint_response_2',
                                                                        'times'],
                                                          points_cb = lambda ud: ud.left_limb_ik_joint_response_1 +
                                                                                 ud.left_limb_ik_joint_response_2),
                               remapping={'times':'left_limb_move_to_pick_object_grip_pose_traj_times'},
                               transitions={'succeeded':'LEFT_LIMB_CLOSE_GRIPPER'})
        
        smach.StateMachine.add('LEFT_LIMB_CLOSE_GRIPPER',
                               GripperInterfaceState(left_gripper_interface, 'close'),
                               transitions={'succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('BAXTER_SMACH_IK_TEST_SERVER', sm, '/SM_ROOT')

    sis.start()
    
    rospy.on_shutdown(lambda: delete_gazebo_model('block'))
    rospy.on_shutdown(lambda: delete_gazebo_model('cafe_table'))

    # left_limb_joint_names = ['left' + joint_name for joint_name in ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']]
    # left_limb_joint_reset_positions = [ 0.19277889178355334,
    #                                     1.0470000630843606,
    #                                    -0.013972419188786667,
    #                                     0.499569185949551,
    #                                    -0.17856443605585426,
    #                                     0.017745921876750614,
    #                                    -0.013274682885986877]
    # left_limb_reset_positions = dict(zip(left_limb_joint_names, left_limb_joint_reset_positions))
    # rospy.on_shutdown(lambda: left_limb_interface.move_to_joint_positions(left_limb_reset_positions))

    # rospy.on_shutdown(right_limb_interface.move_to_neutral)
   
    # Wait for joints to finish reseting
    # rospy.on_shutdown(lambda: rospy.sleep(5))

    # Disable robot
    rospy.on_shutdown(rs.disable)

    outcome = sm.execute()
    
    print("Baxter SMACH Inverse Kinematics Service Test Complete. Ctrl-C to exit.")
    
    rospy.spin()

if __name__ == "__main__":
    main()
