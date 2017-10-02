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
    {{ execute | indent(4) }}
    
    # Disable robot
    rospy.on_shutdown(rs.disable)

    try:
        outcome = sm.execute()
        
        print("Baxter SMACH Pick and Place Test Complete. Ctrl-C to exit.")
    
        rospy.spin()
    except Exception as e:
        rospy.logerr('Error when executing state machine: ' + repr(e))
        rospy.signal_shutdown('Error when executing state machine: ' + repr(e))
{% endblock execute %}

{% block spin %}
{% endblock spin %}
