{% from "Utils.tpl" import render_input_keys, render_output_keys, render_transitions, render_remapping %}

{% include "State.tpl" %}

{% block imports %}
{% if 'import_struct' not in defined_headers %}
import struct
{% do defined_headers.append('import_struct') %}
{% endif %}
{% if 'import_rospy' not in defined_headers %}
import rospy
{% do defined_headers.append('import_rospy') %}
{% endif %}
{% if 'std_msgs_msg_import_Header' not in defined_headers %}
from std_msgs.msg import Header
{% do defined_headers.append('std_msgs_msg_import_Header') %}
{% endif %}
{% if 'baxter_core_msgs_srv_import' not in defined_headers %}
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
{% do defined_headers.append('baxter_core_msgs_srv_import') %}
{% endif %}
{% if 'geometry_msgs_msg_import' not in defined_headers %}
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
{% do defined_headers.append('geometry_msgs_msg_import') %}
{% endif %}
{% endblock imports %}

{% block class_defs %}
{% if 'class_PoseToJointTrajServiceState' not in defined_headers %}
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
{% do defined_headers.append('class_PoseToJointTrajServiceState') %}
{% endif %}
{% endblock class_defs %}

{% block main_def %}
{% if 'ik_service_proxies' not in defined_headers %}
print("Initializing inverse kinematics service proxies for each limb... ")
ik_service_proxies = dict()
ik_service_proxies['left'] = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
ik_service_proxies['right'] = rospy.ServiceProxy('/ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
{% do defined_headers.append('ik_service_proxies') %}
{% endif %}
{% endblock main_def %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
{{ '' | indent(23, true) }}PoseToJointTrajServiceState(ik_service_proxies{% if timeout is defined %}, timeout = {{ timeout }}{% endif %}{% if input_keys is defined %},
{{ render_input_keys(input_keys) }}{% endif %}{% if output_keys is defined %},
{{ render_output_keys(output_keys) }}{% endif %}{% if poses_cb is defined %},
{{ 'poses_cb = ' | indent(51, true) }}{{ poses_cb }}{% endif %}{% if offsets_cb is defined %},
{{ 'offsets_cb = ' | indent(51, true) }}{{ offsets_cb }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
