{% from "Utils.tpl" import render_input_keys, render_transitions, render_remapping %}

{% include "State.tpl" %}

{% block imports %}
{% if 'sensor_msgs_msg_import_JointState' not in defined_headers %}
from sensor_msgs.msg import JointState
{% do defined_headers.append('sensor_msgs_msg_import_JointState') %}
{% endif %}
{% endblock imports %}

{% block class_defs %}
{% if 'class_MoveToJointPositionsState' not in defined_headers %}
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
                limb_joint_names = [self._limb_interfaces[limb].name + joint_name for joint_name in ['_s0', '_s1', '_e0', '_e1', '_w0', '_w1', '_w2']]
                positions = dict(zip(limb_joint_names, positions))
            elif isinstance(positions, JointState):
                positions = dict(zip(positions.name, positions.position))
            else:
                return 'aborted'

            self._limb_interfaces[limb].move_to_joint_positions(positions, timeout=self._timeout)
        else:
            return 'aborted'

        return 'succeeded'
{% do defined_headers.append('class_MoveToJointPositionsState') %}
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
{{ '' | indent(23, true) }}MoveToJointPositionsState(limb_interfaces{% if timeout is defined %}, timeout = {{ timeout }}{% endif %}{% if input_keys is defined %},
{{ render_input_keys(input_keys) }}{% endif %}{% if positions_cb is defined %}, positions_cb = {{ positions_cb }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
