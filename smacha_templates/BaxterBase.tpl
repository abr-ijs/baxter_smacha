{% extends "Base.tpl" %}

{% block imports %}
{{ super() }}
import rospkg

{% if 'geometry_msgs_msg_import' not in defined_headers %}
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
{% do defined_headers.append('geometry_msgs_msg_import') %}
{% endif %}

import baxter_interface
from baxter_interface import CHECK_VERSION
{% endblock imports %}

{% block main_def %}
{{ super () }}
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
{% endblock main_def %}

{% block execute %}
    # Disable robot
    rospy.on_shutdown(rs.disable)
{{ super() }}
{% endblock execute %}
