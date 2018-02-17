{% from "Utils.tpl" import render_transitions, render_remapping, import_geometry_msg %}

{% include "State.tpl" %}

{% block imports %}
{{ import_geometry_msg(defined_headers, 'Pose') }}
{{ import_geometry_msg(defined_headers, 'PoseStamped') }}
{{ import_geometry_msg(defined_headers, 'Point') }}
{{ import_geometry_msg(defined_headers, 'Quaternion') }}
{% endblock imports %}

{% block class_defs %}
{% if 'class_ReadEndpointPoseState' not in defined_headers %}
class ReadEndpointPoseState(smach.State):
    def __init__(self, limb_interfaces, output_keys = ['pose']):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['limb'], output_keys=output_keys)
        self._limb_interfaces = limb_interfaces

    def execute(self, userdata):
        # Get limb from userdata
        if 'limb' in userdata:
            limb = userdata.limb
        else:
            raise ValueError('Limb should be specified in userdata!')

        try:
            current_pose = self._limb_interfaces[limb].endpoint_pose()
        except Exception as e:
            rospy.logerr('Error when reading endpoint pose from limb interface: ' + repr(e))
            raise

        try:
            pose = Pose()
            pose.position.x = current_pose['position'].x
            pose.position.y = current_pose['position'].y
            pose.position.z = current_pose['position'].z
            pose.orientation.x = current_pose['orientation'].x
            pose.orientation.y = current_pose['orientation'].y
            pose.orientation.z = current_pose['orientation'].z
            pose.orientation.w = current_pose['orientation'].w

            userdata.pose = pose
        except Exception as e:
            rospy.logerr('Error when parsing endpoint pose: ' + repr(e))
            raise

        return 'succeeded'
{% do defined_headers.append('class_ReadEndpointPoseState') %}
{% endif %}
{% endblock class_defs %}

{% block main_def %}
{% if 'limb_interfaces' not in defined_headers %}
print("Initializing interfaces for each limb... ")
limb_interfaces = dict()
limb_interfaces['left'] = baxter_interface.Limb('left')
limb_interfaces['right'] = baxter_interface.Limb('right')
{% do defined_headers.append('limb_interfaces') %}
{% endif %}
{% endblock main_def %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
{{ '' | indent(23, true) }}ReadEndpointPoseState(limb_interfaces){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
