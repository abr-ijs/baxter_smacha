{% from "Utils.tpl" import render_transitions, render_remapping %}

{% include "State.tpl" %}

{% block class_defs %}
{% if 'class_GripperInterfaceState' not in defined_headers %}
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
{% do defined_headers.append('class_GripperInterfaceState') %}
{% endif %}
{% endblock class_defs %}

{% block main_def %}
{% if 'gripper_interfaces' not in defined_headers %}
print("Initializing interfaces for each gripper... ")
gripper_interfaces = dict()
gripper_interfaces['left'] = baxter_interface.Gripper('left')
gripper_interfaces['right'] = baxter_interface.Gripper('right')
{% do defined_headers.append('gripper_interfaces') %}
{% endif %}
{% endblock main_def %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
{{ '' | indent(23, true) }}GripperInterfaceState(gripper_interfaces){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
