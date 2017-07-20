#!/usr/bin/env python

import struct
import os

import rospy
import rospkg

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

from sensor_msgs.msg import JointState

import smach
import smach_ros

import baxter_interface
from baxter_interface import CHECK_VERSION


class MoveToJointPositionsState(smach.State):
    """ This state moves a given limb to the specified joint positions using the Baxter inteface.

        Note that this state does not make use of the joint trajectory action server.
    """
    def __init__(self, limb_interface, timeout=15.0, input_keys = ['positions'], positions_cb = None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys)
        self._limb_interface = limb_interface
        self._timeout = timeout
        
        # Set up a points callback
        self._positions_cb = positions_cb

    def execute(self, userdata):
        # If a positions callback has been defined, use it to format
        # positions specified by the input keys in the userdata
        if self._positions_cb:
            positions = self._positions_cb(userdata)
        else:
            positions = userdata.positions

        # Check whether or not positions is a singleton and convert if necessary
        if isinstance(positions, list):
            if isinstance(positions[0], list) or isinstance(positions[0], JointState):
                positions = positions[0]
                
        # Parse positions
        if positions:
            if isinstance(positions, list):
                limb_joint_names = [self._limb_interface.name + joint_name for joint_name in ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']]
                positions = dict(zip(limb_joint_names, positions))
            elif isinstance(positions, JointState):
                positions = dict(zip(positions.name, positions.position))
            else:
                return 'aborted'

            self._limb_interface.move_to_joint_positions(positions, timeout=self._timeout)
        else:
            return 'aborted'

        return 'succeeded'


class PoseToJointTrajServiceState(smach.State):
    def __init__(self, ik_service_proxy, timeout=5.0,
                 input_keys=['poses', 'offsets'], output_keys=['joints'], poses_cb = None, offsets_cb = None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys, output_keys=output_keys)
        self._ik_service_proxy = ik_service_proxy
        self._timeout = timeout
        self._verbose = True
        
        # Set up a poses callback
        self._poses_cb = poses_cb

        # Set up an offsets callback
        self._offsets_cb = offsets_cb

    def execute(self, userdata):
        # If a poses callback has been defined, use it to format
        # poses specified by the input keys in the userdata
        if self._poses_cb:
            poses = self._poses_cb(userdata)
        else:
            poses = userdata.poses
        
        # If an offsets callback has been defined, use it to format
        # offsets specified by the input keys in the userdata
        if self._offsets_cb:
            offsets = self._offsets_cb(userdata)
        else:
            if 'offsets' in userdata:
                offsets = userdata.offsets
            else:
                offsets = None

        # Check if poses is a singleton and convert to list if necessary
        if not isinstance(poses, list):
            poses = [poses]
        elif len(poses) == 2 and len(poses[0]) != len(poses[1]):
            poses = [poses]
        else:
            return 'aborted'

        # Check if offsets is a singleton and convert to list if necessary
        if offsets:
            if not isinstance(offsets, list):
                offsets = [offsets]
            elif len(offsets) == 2 and len(offsets[0]) != len(offsets[1]):
                offsets = [offsets]
            else:
                return 'aborted'
        
        # Set up a request object
        ik_request = SolvePositionIKRequest()

        # Parse poses from userdata, stamp them, add offsets if required,
        # and append to inverse kinematics request.
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        for i_pose in range(len(poses)):
            # Parse pose
            pose = poses[i_pose]
            if isinstance(pose, PoseStamped):
                pose_stamped = pose
            elif isinstance(pose, Pose):
                pose_stamped = PoseStamped(header=header, pose=pose)
            elif isinstance(pose, list):
                position = Point(x=pose[0][0], y=pose[0][1], z=pose[0][2])
                orientation = Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3])
                pose_stamped = PoseStamped(header=header, pose = Pose(position=position, orientation=orientation))
            else:
                return 'aborted'

            # Parse offset
            if offsets:
                offset = offsets[i_pose]
                if isinstance(offset, PoseStamped):
                    offset = offset.pose
                elif isinstance(offset, Pose):
                    pass
                elif isinstance(offset, list):
                    offset = Pose(position=Point(x=offset[0][0], y=offset[0][1], z=offset[0][2]),
                                  orientation=Quaternion(x=offset[1][0], y=offset[1][1], z=offset[1][2], w=offset[1][3]))

                pose_stamped.pose.position.x = pose_stamped.pose.position.x + offset.position.x
                pose_stamped.pose.position.y = pose_stamped.pose.position.y + offset.position.y
                pose_stamped.pose.position.z = pose_stamped.pose.position.z + offset.position.z
                pose_stamped.pose.orientation.x = pose_stamped.pose.orientation.x + offset.orientation.x
                pose_stamped.pose.orientation.y = pose_stamped.pose.orientation.y + offset.orientation.y
                pose_stamped.pose.orientation.z = pose_stamped.pose.orientation.z + offset.orientation.z
                pose_stamped.pose.orientation.w = pose_stamped.pose.orientation.w + offset.orientation.w

            # Append pose to IK request
            ik_request.pose_stamp.append(pose_stamped)

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

        limb_joints = {}
        if (resp_seeds[0] != ik_response.RESULT_INVALID):
            seed_str = {
                        ik_request.SEED_USER: 'User Provided Seed',
                        ik_request.SEED_CURRENT: 'Current Joint Angles',
                        ik_request.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return 'aborted'

        if any(response == ik_response.RESULT_INVALID for response in resp_seeds):
            print('resp_seeds: {}'.format(resp_seeds))
            return 'aborted'
        else:
            userdata.joints = ik_response.joints
            return 'succeeded'


class LoadGazeboModelState(smach.State):
    def __init__(self, name, model_path, input_keys = ['pose', 'reference_frame'], pose_cb = None):
        smach.State.__init__(self, input_keys=input_keys, outcomes=['succeeded'])

        self._name = name
        self._model_path = model_path
        
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
        with open(self._model_path, 'r') as model_file:
            model_xml = model_file.read().replace('\n', '')

        # Spawn model SDF/URDF
        if os.path.splitext(self._model_path)[1][1:].lower() == 'sdf':
            spawn_service_type = 'sdf'
        elif os.path.splitext(self._model_path)[1][1:].lower() == 'urdf':
            spawn_service_type = 'urdf'

        try:
            spawn_service_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_response = spawn_service_proxy(self._name, model_xml, "/",
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


class ReadEndpointPoseState(smach.State):
    def __init__(self, limb_interface, output_keys = ['pose']):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=output_keys)
        self._limb_interface = limb_interface

    def execute(self, userdata):
        current_pose = self._limb_interface.endpoint_pose()

        pose = Pose()
        pose.position.x = current_pose['position'].x
        pose.position.y = current_pose['position'].y
        pose.position.z = current_pose['position'].z
        pose.orientation.x = current_pose['orientation'].x
        pose.orientation.y = current_pose['orientation'].y
        pose.orientation.z = current_pose['orientation'].z
        pose.orientation.w = current_pose['orientation'].w

        userdata.pose = pose

        return 'succeeded'


def main():
    print("Starting Baxter SMACH Pick and Place Test.")
    
    print("Initializing node...")
    rospy.init_node('baxter_smach_pick_and_place_test')
    
    print("Initializing interfaces for each limb... ")
    left_limb_interface = baxter_interface.Limb('left')
    right_limb_interface = baxter_interface.Limb('right')
    
    print("Initializing interfaces for each gripper... ")
    left_gripper_interface = baxter_interface.Gripper('left')
    right_gripper_interface = baxter_interface.Gripper('right')
        
    print("Initializing inverse kinematics service proxies for each limb... ")
    left_limb_ik_service_proxy = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
    right_limb_ik_service_proxy = rospy.ServiceProxy('/ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        

    with sm:
    
        sm.userdata.overhead_orientation = [-0.0249590815779, 0.999649402929, 0.00737916180073, 0.00486450832011]
        sm.userdata.hover_offset = [[0.0, 0.0, 0.15], [0.0, 0.0, 0.0, 0.0]]

        sm.userdata.table_model_pose_world = Pose(position=Point(x=1.0, y=0.0, z=0.0))
        sm.userdata.table_model_ref_frame = 'world'
            
        smach.StateMachine.add('LOAD_TABLE_MODEL',
                               LoadGazeboModelState('cafe_table',
                                                    rospkg.RosPack().get_path('baxter_sim_examples')+'/models/cafe_table/model.sdf'),
                               remapping={'pose':'table_model_pose_world',
                                          'reference_frame': 'table_model_ref_frame'},
                               transitions={'succeeded':'LOAD_BLOCK_MODEL'})
        
        sm.userdata.block_model_pick_pose_world = [[0.6725, 0.1265, 0.7825], [0.0, 0.0, 0.0, 0.0]]
        sm.userdata.block_model_pick_ref_frame = 'world'
        sm.userdata.block_model_pick_pose = [[0.7, 0.15, -0.129], sm.userdata.overhead_orientation]
        sm.userdata.block_model_place_pose = [[0.75, 0.0, -0.129], sm.userdata.overhead_orientation]
        
        smach.StateMachine.add('LOAD_BLOCK_MODEL',
                               LoadGazeboModelState('block',
                                                    rospkg.RosPack().get_path('baxter_sim_examples')+'/models/block/model.urdf'),
                               remapping={'pose':'block_model_pick_pose_world',
                                          'reference_frame':'block_model_pick_ref_frame'},
                               transitions={'succeeded':'MOVE_TO_START_POSITION'})

        sm.userdata.joint_start_positions = (
                [-0.08000397926829805,
                 -0.9999781166910306,
                 -1.189968899785275,
                  1.9400238130755056,
                  0.6699952259595108,
                  1.030009435085784,
                 -0.4999997247485215] )
        
        smach.StateMachine.add('MOVE_TO_START_POSITION',
                               MoveToJointPositionsState(left_limb_interface),
                               remapping={'positions':'joint_start_positions'},
                               transitions={'succeeded':'PICK_BLOCK'})
            
        sm_pick_block = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                                           input_keys=['block_model_pick_pose',
                                                       'overhead_orientation',
                                                       'hover_offset',
                                                       'joint_start_positions'],
                                           output_keys=['ik_joint_response_block_pick_hover_pose',
                                                        'ik_joint_response_block_pick_pose'])

        with sm_pick_block:

            smach.StateMachine.add('IK_PICK_BLOCK_HOVER_POSE',
                                   PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                                   remapping={'poses':'block_model_pick_pose',
                                              'offsets':'hover_offset',
                                              'joints':'ik_joint_response_block_pick_hover_pose'},
                                   transitions={'succeeded':'MOVE_TO_PICK_BLOCK_HOVER_POSE'})
            
            smach.StateMachine.add('MOVE_TO_PICK_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(left_limb_interface),
                                   remapping={'positions':'ik_joint_response_block_pick_hover_pose'},
                                   transitions={'succeeded':'OPEN_GRIPPER'})

            smach.StateMachine.add('OPEN_GRIPPER',
                                   GripperInterfaceState(left_gripper_interface, 'open'),
                                   transitions={'succeeded':'IK_PICK_BLOCK_GRIP_POSE'})
            
            smach.StateMachine.add('IK_PICK_BLOCK_GRIP_POSE',
                                   PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                                   remapping={'poses':'block_model_pick_pose',
                                              'joints':'ik_joint_response_block_pick_pose'},
                                   transitions={'succeeded':'MOVE_TO_PICK_BLOCK_GRIP_POSE'})
            
            smach.StateMachine.add('MOVE_TO_PICK_BLOCK_GRIP_POSE',
                                   MoveToJointPositionsState(left_limb_interface),
                                   remapping={'positions':'ik_joint_response_block_pick_pose'},
                                   transitions={'succeeded':'CLOSE_GRIPPER'})
            
            smach.StateMachine.add('CLOSE_GRIPPER',
                                   GripperInterfaceState(left_gripper_interface, 'close'),
                                   transitions={'succeeded':'READ_GRIPPED_BLOCK_ENDPOINT_POSE'})

            smach.StateMachine.add('READ_GRIPPED_BLOCK_ENDPOINT_POSE',
                                   ReadEndpointPoseState(left_limb_interface),
                                   remapping={'pose':'gripped_block_endpoint_pose'},
                                   transitions={'succeeded':'IK_PICK_GRIPPED_BLOCK_HOVER_POSE'})
            
            smach.StateMachine.add('IK_PICK_GRIPPED_BLOCK_HOVER_POSE',
                                   PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                                   remapping={'poses':'gripped_block_endpoint_pose',
                                              'offsets':'hover_offset',
                                              'joints':'ik_joint_response_gripped_block_hover_pose'},
                                   transitions={'succeeded':'MOVE_TO_GRIPPED_BLOCK_HOVER_POSE'})
            
            smach.StateMachine.add('MOVE_TO_GRIPPED_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(left_limb_interface),
                                   remapping={'positions':'ik_joint_response_gripped_block_hover_pose'},
                                   transitions={'succeeded':'succeeded'})
        
            
        smach.StateMachine.add('PICK_BLOCK', sm_pick_block,
                               transitions={'succeeded':'PLACE_BLOCK'}) 
        
        sm_place_block = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                                            input_keys=['block_model_place_pose',
                                                        'overhead_orientation',
                                                        'hover_offset',
                                                        'ik_joint_response_block_pick_hover_pose',
                                                        'ik_joint_response_block_pick_pose'])

        with sm_place_block:
        
            smach.StateMachine.add('IK_PLACE_BLOCK_HOVER_POSE',
                                   PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                                   remapping={'poses':'block_model_place_pose',
                                              'offsets':'hover_offset',
                                              'joints':'ik_joint_response_block_place_hover_pose'},
                                   transitions={'succeeded':'MOVE_TO_PLACE_BLOCK_HOVER_POSE'})
            
            smach.StateMachine.add('MOVE_TO_PLACE_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(left_limb_interface),
                                   remapping={'positions':'ik_joint_response_block_place_hover_pose'},
                                   transitions={'succeeded':'IK_PLACE_BLOCK_RELEASE_POSE'})
            
            smach.StateMachine.add('IK_PLACE_BLOCK_RELEASE_POSE',
                                   PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                                   remapping={'poses':'block_model_place_pose',
                                              'joints':'ik_joint_response_block_place_pose'},
                                   transitions={'succeeded':'MOVE_TO_PLACE_BLOCK_RELEASE_POSE'})
            
            smach.StateMachine.add('MOVE_TO_PLACE_BLOCK_RELEASE_POSE',
                                   MoveToJointPositionsState(left_limb_interface),
                                   remapping={'positions':'ik_joint_response_block_place_pose'},
                                   transitions={'succeeded':'OPEN_GRIPPER'})
            
            smach.StateMachine.add('OPEN_GRIPPER',
                                   GripperInterfaceState(left_gripper_interface, 'open'),
                                   transitions={'succeeded':'IK_PLACE_BLOCK_RELEASED_HOVER_POSE'})
            
            smach.StateMachine.add('IK_PLACE_BLOCK_RELEASED_HOVER_POSE',
                                   PoseToJointTrajServiceState(left_limb_ik_service_proxy),
                                   remapping={'poses':'block_model_place_pose',
                                              'offsets':'hover_offset',
                                              'joints':'ik_joint_response_block_released_hover_pose'},
                                   transitions={'succeeded':'MOVE_TO_RELEASED_BLOCK_HOVER_POSE'})
            
            smach.StateMachine.add('MOVE_TO_RELEASED_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(left_limb_interface),
                                   remapping={'positions':'ik_joint_response_block_released_hover_pose'},
                                   transitions={'succeeded':'succeeded'})
        
        smach.StateMachine.add('PLACE_BLOCK', sm_place_block,
                               transitions={'succeeded':'succeeded'}) 
        
        

    sis = smach_ros.IntrospectionServer('BAXTER_SMACH_IK_TEST_SERVER', sm, '/SM_ROOT')

    sis.start()
    
    rospy.on_shutdown(lambda: delete_gazebo_model('block'))
    rospy.on_shutdown(lambda: delete_gazebo_model('cafe_table'))

    # Disable robot
    rospy.on_shutdown(rs.disable)

    outcome = sm.execute()
    
    print("Baxter SMACH Pick and Place Test Complete. Ctrl-C to exit.")
    
    rospy.spin()

if __name__ == "__main__":
    main()
