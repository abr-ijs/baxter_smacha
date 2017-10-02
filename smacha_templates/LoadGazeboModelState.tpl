{% from "Utils.tpl" import render_transitions, render_remapping %}

{% include "State.tpl" %}

{% block imports %}
{% if 'import_os' not in defined_headers %}
import os
{% do defined_headers.append('import os') %}
{% endif %}
{% if 'gazebo_msgs_srv_import' not in defined_headers %}
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
{% do defined_headers.append('gazebo_msgs_srv_import') %}
{% endif %}
{% endblock imports %}

{% block defs %}
{% if 'def_delete_gazebo_model' not in defined_headers %}
def delete_gazebo_model(model):
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model(model)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
{% do defined_headers.append('def_delete_gazebo_model') %}
{% endif %}
{% endblock defs %}

{% block class_defs %}
{% if 'class_LoadGazeboModelState' not in defined_headers %}
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
{% do defined_headers.append('class_LoadGazeboModelState') %}
{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
{{ '' | indent(23, true) }}LoadGazeboModelState({% if model_name is not_string %}{{ model_name }}{% else %}'{{ model_name }}'{% endif %},
                                                {% if model_path is not_string %}{{ model_path }}{% else %}'{{ model_path }}'{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}

{% block execute %}
rospy.on_shutdown(lambda: delete_gazebo_model('{{ model_name }}'))
{% endblock execute %}
