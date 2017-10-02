#!/usr/bin/env python





import roslib; 
import rospy
import smach
import smach_ros
import os



from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import os

from sensor_msgs.msg import JointState

import struct



import rospy



from std_msgs.msg import Header



from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import rospkg


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)



import baxter_interface
from baxter_interface import CHECK_VERSION



def delete_gazebo_model(model):
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model(model)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))



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
            try:
                pose = self._pose_cb(userdata)
            except Exception as e:
                rospy.logerr('Error when using poses callback to format poses: ' + repr(e))
                raise
        else:
            if 'pose' in userdata:
                pose = userdata.pose
            else:
                raise ValueError('Pose should be specified in userdata!')

        # Parse pose
        try:
            if isinstance(pose, PoseStamped):
                pose = pose.pose
            elif isinstance(pose, Pose):
                pose = pose
            elif isinstance(pose, list):
                position = Point(x=pose[0][0], y=pose[0][1], z=pose[0][2])
                orientation = Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3])
                pose = Pose(position=position, orientation=orientation)
            else:
                raise ValueError('Pose should be specified as a list, Pose or PoseStamped!')
        except Exception as e:
            rospy.logerr('Error when parsing Gazebo model pose: ' + repr(e))
            raise

        # Parse reference_frame
        try:
            if 'reference_frame' in userdata:
                reference_frame = userdata.reference_frame

                if isinstance(reference_frame, str):
                    pass
                elif isinstance(reference_frame, list):
                    if isinstance(reference_frame[0], str):
                        reference_frame = reference_frame[0]
                    else:
                        raise ValueError('The reference frame should be specified as a string!')
                else:
                        raise ValueError('The reference frame should be specified as a string!')
            else:
                raise ValueError('The reference frame should be specified in userdata!')
        except Exception as e:
            rospy.logerr('Error when parsing Gazebo model reference frame: ' + repr(e))
            raise

        # Load model SDF/URDF XML
        try:
            model_xml = ''
            with open(self._model_path, 'r') as model_file:
                model_xml = model_file.read().replace('\n', '')
        except Exception as e:
            rospy.logerr('Error when loading Gazebo model XML file: ' + repr(e))
            raise

        # Spawn model SDF/URDF
        try:
            if os.path.splitext(self._model_path)[1][1:].lower() == 'sdf':
                spawn_service_type = 'sdf'
            elif os.path.splitext(self._model_path)[1][1:].lower() == 'urdf':
                spawn_service_type = 'urdf'
        except Exception as e:
            rospy.logerr('Error when determining whether Gazebo model is SDF or URDF: ' + repr(e))
            raise

        try:
            spawn_service_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_response = spawn_service_proxy(self._name, model_xml, "/",
                                                 pose, reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr('Spawn ' + spawn_service_type.upper() + ' service call failed: {0}'.format(e))

        return 'succeeded'

class MoveToJointPositionsState(smach.State):
    """ This state moves a given limb to the specified joint positions using the Baxter inteface.

        Note that this state does not make use of the joint trajectory action server.
    """
    def __init__(self, limb_interfaces, timeout=15.0, input_keys = ['limb', 'positions'], positions_cb = None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys)
        self._limb_interfaces = limb_interfaces
        self._timeout = timeout
        
        # Set up a points callback
        self._positions_cb = positions_cb

    def execute(self, userdata):
        # Get limb from userdata
        limb = userdata.limb

        # If a positions callback has been defined, use it to format
        # positions specified by the input keys in the userdata
        if self._positions_cb:
            try:
                positions = self._positions_cb(userdata)
            except Exception as e:
                rospy.logerr('Error when using positions callback to format joint positions: ' + repr(e))
                raise
        else:
            if 'positions' in userdata:
                positions = userdata.positions
            else:
                raise ValueError('Joint positions should be specified in userdata!')

        # Check whether or not positions is a singleton and convert if necessary
        try:
            if isinstance(positions, list):
                if isinstance(positions[0], list) or isinstance(positions[0], JointState):
                    positions = positions[0]
        except Exception as e:
            rospy.logerr('Error when converting joint positions to singleton: ' + repr(e))
            raise
                
        # Parse positions
        try:
            if isinstance(positions, list):
                limb_joint_names = [self._limb_interfaces[limb].name + joint_name for joint_name in ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']]
                positions = dict(zip(limb_joint_names, positions))
            elif isinstance(positions, JointState):
                positions = dict(zip(positions.name, positions.position))
            else:
                raise ValueError('Positions should be specified as a list or a JointState.')
        except Exception as e:
            rospy.logerr('Error when parsing joint positions: ' + repr(e))
            raise

        try:
            self._limb_interfaces[limb].move_to_joint_positions(positions, timeout=self._timeout)
        except Exception as e:
            rospy.logerr('Error when using limb interface to move to joint positions: ' + repr(e))
            raise

        return 'succeeded'

class PoseToJointTrajServiceState(smach.State):
    def __init__(self, ik_service_proxies, timeout=5.0,
                 input_keys=['limb', 'poses', 'offsets'], output_keys=['joints'], poses_cb = None, offsets_cb = None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys, output_keys=output_keys)
        self._ik_service_proxies = ik_service_proxies
        self._timeout = timeout
        self._verbose = True
        
        # Set up a poses callback
        self._poses_cb = poses_cb

        # Set up an offsets callback
        self._offsets_cb = offsets_cb

    def execute(self, userdata):
        # Get limb from userdata
        limb = userdata.limb

        # If a poses callback has been defined, use it to format
        # poses specified by the input keys in the userdata
        if self._poses_cb:
            try:
                poses = self._poses_cb(userdata)
            except Exception as e:
                rospy.logerr('Error when using poses callback to format poses: ' + repr(e))
                raise
        else:
            if 'poses' in userdata:
                poses = userdata.poses
            else:
                raise ValueError('Joint positions should be specified in userdata!')
        
        # If an offsets callback has been defined, use it to format
        # offsets specified by the input keys in the userdata
        if self._offsets_cb:
            try:
                offsets = self._offsets_cb(userdata)
            except Exception as e:
                rospy.logerr('Error when using offsets callback to format pose offsets: ' + repr(e))
                raise
        else:
            if 'offsets' in userdata:
                offsets = userdata.offsets
            else:
                offsets = None

        # Check if poses is a list, singleton or otherwise, and convert if necessary
        try:
            if not isinstance(poses, list):
                poses = [poses]
            elif len(poses) == 2 and len(poses[0]) != len(poses[1]):
                poses = [poses]
            else:
                raise ValueError('Poses should be specified as a list!')
        except Exception as e:
            rospy.logerr('Error when converting poses to a list: ' + repr(e))
            raise

        # Check if offsets is a singleton and convert to list if necessary
        if offsets:
            try:
                if not isinstance(offsets, list):
                    offsets = [offsets]
                elif len(offsets) == 2 and len(offsets[0]) != len(offsets[1]):
                    offsets = [offsets]
                else:
                    raise ValueError('Offsets should be specified as a list!')
            except Exception as e:
                rospy.logerr('Error when converting offsets to a list: ' + repr(e))
                raise
        
        # Set up a request object
        ik_request = SolvePositionIKRequest()

        # Parse poses from userdata, stamp them, add offsets if required,
        # and append to inverse kinematics request.
        try:
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
        except Exception as e:
            rospy.logerr('Error when parsing poses/offsets and building inverse kinematics request: ' + repr(e))
            raise


        # Wait for service (important!)
        self._ik_service_proxies[limb].wait_for_service(self._timeout)

        # Receive response
        try:
            ik_response = self._ik_service_proxies[limb](ik_request)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Inverse kinematics service call failed: %s" % (e,))
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

class GripperInterfaceState(smach.State):
    def __init__(self, gripper_interfaces):
        smach.State.__init__(self, input_keys = ['limb', 'command'], outcomes=['succeeded', 'aborted'])
        self._gripper_interfaces = gripper_interfaces

    def execute(self, userdata):
        # Get limb + command from userdata
        if 'limb' in userdata:
            limb = userdata.limb
        else:
            raise ValueError('Limb should be specified in userdata!')
        if 'command' in userdata:
            command = userdata.command
        else:
            raise ValueError('Command should be specified in userdata!')

        # Parse command
        try:
            if isinstance(command, str):
                pass
            elif isinstance(command, list):
                if isinstance(command[0], str):
                    command = command[0]
                else:
                    raise ValueError('Command should be specified as a string!')
            else:
                    raise ValueError('Command should be specified as a string!')
        except Exception as e:
            rospy.logerr('Error when parsing gripper interface command: ' + repr(e))
            raise

        try:
            if command == 'open':
                self._gripper_interfaces[limb].open()
                rospy.sleep(1.0)
            elif command == 'close':
                self._gripper_interfaces[limb].close()
                rospy.sleep(1.0)
            else:
                raise ValueError('Command should be either \'open\' or \'close\'!')
        except Exception as e:
            rospy.logerr('Error when running gripper interface command: ' + repr(e))
            raise

        return 'succeeded'






def main():
    rospy.init_node('baxter_smach_pick_and_place_test')

    print("Initializing interfaces for each limb... ")
    limb_interfaces = dict()
    limb_interfaces['left'] = baxter_interface.Limb('left')
    limb_interfaces['right'] = baxter_interface.Limb('right')
    
    print("Initializing inverse kinematics service proxies for each limb... ")
    ik_service_proxies = dict()
    ik_service_proxies['left'] = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
    ik_service_proxies['right'] = rospy.ServiceProxy('/ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
    
    print("Initializing interfaces for each gripper... ")
    gripper_interfaces = dict()
    gripper_interfaces['left'] = baxter_interface.Gripper('left')
    gripper_interfaces['right'] = baxter_interface.Gripper('right')

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

   

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])


    
    sm.userdata.hover_offset = [[0.0, 0.0, 0.15], [0.0, 0.0, 0.0, 0.0]]
    
    sm.userdata.limb = 'left'
    
    sm.userdata.table_model_pose_world = Pose(position=Point(x=1.0, y=0.0, z=0.0))
    
    sm.userdata.table_model_ref_frame = 'world'
    
    sm.userdata.block_model_pick_pose = [[0.7, 0.15, -0.129], [-0.02496, 0.99965, 0.00738, 0.00486]]
    
    sm.userdata.block_model_pick_pose_world = [[0.6725, 0.1265, 0.7825], [0.0, 0.0, 0.0, 0.0]]
    
    sm.userdata.block_model_pick_ref_frame = 'world'
    
    sm.userdata.block_model_place_pose = [[0.75, 0.0, -0.129], [-0.02496, 0.99965, 0.00738, 0.00486]]
    
    sm.userdata.joint_start_positions = [-0.08, -0.99998, -1.18997, 1.94002, 0.67, 1.03001, -0.5]

    with sm:

        smach.StateMachine.add('LOAD_TABLE_MODEL',
                               LoadGazeboModelState('cafe_table',
        rospkg.RosPack().get_path('baxter_sim_examples')+'/models/cafe_table/model.sdf'),
                               transitions={'succeeded':'LOAD_BLOCK_MODEL'},
                               remapping={'pose':'table_model_pose_world',
                                          'reference_frame':'table_model_ref_frame'})
        
        smach.StateMachine.add('LOAD_BLOCK_MODEL',
                               LoadGazeboModelState('block',
        rospkg.RosPack().get_path('baxter_sim_examples')+'/models/block/model.urdf'),
                               transitions={'succeeded':'MOVE_TO_START_POSITION'},
                               remapping={'pose':'block_model_pick_pose_world',
                                          'reference_frame':'block_model_pick_ref_frame'})
        
        smach.StateMachine.add('MOVE_TO_START_POSITION',
                               MoveToJointPositionsState(limb_interfaces),
                               transitions={'succeeded':'PICK_BLOCK'},
                               remapping={'limb':'limb',
                                          'positions':'joint_start_positions'})
        
        sm_pick_block = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                                             input_keys = ['limb', 'pick_pose', 'hover_offset'])
        
        
        
        sm_pick_block.userdata.close_command = 'close'
        
        sm_pick_block.userdata.open_command = 'open'
        
        
        
        
        with sm_pick_block:
        
            smach.StateMachine.add('IK_PICK_BLOCK_HOVER_POSE',
                                   PoseToJointTrajServiceState(ik_service_proxies),
                                   transitions={'succeeded':'MOVE_TO_PICK_BLOCK_HOVER_POSE'},
                                   remapping={'joints':'ik_joint_response_block_pick_hover_pose',
                                              'limb':'limb',
                                              'offsets':'hover_offset',
                                              'poses':'pick_pose'})
            
            smach.StateMachine.add('MOVE_TO_PICK_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(limb_interfaces),
                                   transitions={'succeeded':'OPEN_GRIPPER'},
                                   remapping={'limb':'limb',
                                              'positions':'ik_joint_response_block_pick_hover_pose'})
            
            smach.StateMachine.add('OPEN_GRIPPER',
                                   GripperInterfaceState(gripper_interfaces),
                                   transitions={'succeeded':'IK_PICK_BLOCK_GRIP_POSE'},
                                   remapping={'command':'open_command',
                                              'limb':'limb'})
            
            smach.StateMachine.add('IK_PICK_BLOCK_GRIP_POSE',
                                   PoseToJointTrajServiceState(ik_service_proxies),
                                   transitions={'succeeded':'MOVE_TO_PICK_BLOCK_GRIP_POSE'},
                                   remapping={'joints':'ik_joint_response_block_pick_pose',
                                              'limb':'limb',
                                              'poses':'pick_pose'})
            
            smach.StateMachine.add('MOVE_TO_PICK_BLOCK_GRIP_POSE',
                                   MoveToJointPositionsState(limb_interfaces),
                                   transitions={'succeeded':'CLOSE_GRIPPER'},
                                   remapping={'limb':'limb',
                                              'positions':'ik_joint_response_block_pick_pose'})
            
            smach.StateMachine.add('CLOSE_GRIPPER',
                                   GripperInterfaceState(gripper_interfaces),
                                   transitions={'succeeded':'MOVE_TO_GRIPPED_BLOCK_HOVER_POSE'},
                                   remapping={'command':'close_command',
                                              'limb':'limb'})
            
            smach.StateMachine.add('MOVE_TO_GRIPPED_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(limb_interfaces),
                                   transitions={'succeeded':'succeeded'},
                                   remapping={'limb':'limb',
                                              'positions':'ik_joint_response_block_pick_hover_pose'})
            
        
        smach.StateMachine.add('PICK_BLOCK', sm_pick_block,
                               transitions={'succeeded':'PLACE_BLOCK'},
                               remapping={'hover_offset':'hover_offset',
                                          'limb':'limb',
                                          'pick_pose':'block_model_pick_pose'})
        
        sm_place_block = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                                             input_keys = ['limb', 'place_pose', 'hover_offset'])
        
        
        
        sm_place_block.userdata.open_command = 'open'
        
        
        
        
        with sm_place_block:
        
            smach.StateMachine.add('IK_PLACE_BLOCK_HOVER_POSE',
                                   PoseToJointTrajServiceState(ik_service_proxies),
                                   transitions={'succeeded':'MOVE_TO_PLACE_BLOCK_HOVER_POSE'},
                                   remapping={'joints':'ik_joint_response_block_place_hover_pose',
                                              'limb':'limb',
                                              'offsets':'hover_offset',
                                              'poses':'place_pose'})
            
            smach.StateMachine.add('MOVE_TO_PLACE_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(limb_interfaces),
                                   transitions={'succeeded':'IK_PLACE_BLOCK_RELEASE_POSE'},
                                   remapping={'limb':'limb',
                                              'positions':'ik_joint_response_block_place_hover_pose'})
            
            smach.StateMachine.add('IK_PLACE_BLOCK_RELEASE_POSE',
                                   PoseToJointTrajServiceState(ik_service_proxies),
                                   transitions={'succeeded':'MOVE_TO_PLACE_BLOCK_RELEASE_POSE'},
                                   remapping={'joints':'ik_joint_response_block_place_pose',
                                              'limb':'limb',
                                              'poses':'place_pose'})
            
            smach.StateMachine.add('MOVE_TO_PLACE_BLOCK_RELEASE_POSE',
                                   MoveToJointPositionsState(limb_interfaces),
                                   transitions={'succeeded':'OPEN_GRIPPER'},
                                   remapping={'limb':'limb',
                                              'positions':'ik_joint_response_block_place_pose'})
            
            smach.StateMachine.add('OPEN_GRIPPER',
                                   GripperInterfaceState(gripper_interfaces),
                                   transitions={'succeeded':'MOVE_TO_RELEASED_BLOCK_HOVER_POSE'},
                                   remapping={'command':'open_command',
                                              'limb':'limb'})
            
            smach.StateMachine.add('MOVE_TO_RELEASED_BLOCK_HOVER_POSE',
                                   MoveToJointPositionsState(limb_interfaces),
                                   transitions={'succeeded':'succeeded'},
                                   remapping={'limb':'limb',
                                              'positions':'ik_joint_response_block_place_hover_pose'})
            
        
        smach.StateMachine.add('PLACE_BLOCK', sm_place_block,
                               transitions={'succeeded':'succeeded'},
                               remapping={'hover_offset':'hover_offset',
                                          'limb':'limb',
                                          'place_pose':'block_model_place_pose'})



        



    sis = smach_ros.IntrospectionServer('baxter_smach_pick_and_place_test', sm, '/sm')
    sis.start()



    rospy.on_shutdown(lambda: delete_gazebo_model('block'))
    
    rospy.on_shutdown(lambda: delete_gazebo_model('cafe_table'))
    
    # Disable robot
    rospy.on_shutdown(rs.disable)

    try:
        outcome = sm.execute()
        
        print("Baxter SMACH Pick and Place Test Complete. Ctrl-C to exit.")
    
        rospy.spin()
    except Exception as e:
        rospy.logerr('Error when executing state machine: ' + repr(e))
        rospy.signal_shutdown('Error when executing state machine: ' + repr(e))
    





    



if __name__ == '__main__':
    main()