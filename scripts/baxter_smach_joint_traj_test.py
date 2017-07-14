#!/usr/bin/env python

# import argparse
import sys

from copy import copy

import rospy
import rostopic

import actionlib

import threading

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

# import baxter_interface

# from baxter_interface import CHECK_VERSION


# class Trajectory(object):
#     def __init__(self, limb):
#         ns = 'robot/limb/' + limb + '/'
#         self._client = actionlib.SimpleActionClient(
#             ns + "follow_joint_trajectory",
#             FollowJointTrajectoryAction,
#         )
#         self._goal = FollowJointTrajectoryGoal()
#         self._goal_time_tolerance = rospy.Time(0.1)
#         self._goal.goal_time_tolerance = self._goal_time_tolerance
#         server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
#         if not server_up:
#             rospy.logerr("Timed out waiting for Joint Trajectory"
#                          " Action Server to connect. Start the action server"
#                          " before running example.")
#             rospy.signal_shutdown("Timed out waiting for Action Server")
#             sys.exit(1)
#         self.clear(limb)
# 
#     def add_point(self, positions, time):
#         point = JointTrajectoryPoint()
#         point.positions = copy(positions)
#         point.time_from_start = rospy.Duration(time)
#         self._goal.trajectory.points.append(point)
# 
#     def start(self):
#         self._goal.trajectory.header.stamp = rospy.Time.now()
#         self._client.send_goal(self._goal)
# 
#     def stop(self):
#         self._client.cancel_goal()
# 
#     def wait(self, timeout=15.0):
#         self._client.wait_for_result(timeout=rospy.Duration(timeout))
# 
#     def result(self):
#         return self._client.get_result()
# 
#     def clear(self, limb):
#         self._goal = FollowJointTrajectoryGoal()
#         self._goal.goal_time_tolerance = self._goal_time_tolerance
#         self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
#             ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

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

class ReadTopicState(WaitForMsgState):
    def __init__(self, topic, msg_type, fields, **kwargs):
        self._fields = fields
        WaitForMsgState.__init__(self, topic, msg_type, msg_cb=self._msg_cb, latch=True, output_keys=fields, **kwargs)

    def _msg_cb(self, msg, ud):
        for field in self._fields:
            field_reader = rostopic.msgevalgen(field)
            setattr(ud, field, field_reader(msg))
        return msg is not None

# class Foo(smach.State):
#      def __init__(self, outcomes=['outcome1', 'outcome2'],
#                         input_keys=['foo_input'],
#                         output_keys=['foo_output'])
# 
#      def execute(self, userdata):
#         # Do something with userdata
#         if userdata.foo_input == 1:
#             return 'outcome1'
#         else:
#             userdata.foo_output = 3
#             return 'outcome2'

class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded'],
                             input_keys=['name', 'position'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        print('name: {}'.format(userdata.name))        
        print('position: {}'.format(userdata.position))        
        return 'succeeded'

def main():
    # """RSDK Joint Trajectory Example: Simple Action Client

    # Creates a client of the Joint Trajectory Action Server
    # to send commands of standard action type,
    # control_msgs/FollowJointTrajectoryAction.

    # Make sure to start the joint_trajectory_action_server.py
    # first. Then run this example on a specified limb to
    # command a short series of trajectory points for the arm
    # to follow.
    # """
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__)
    # required = parser.add_argument_group('required arguments')
    # required.add_argument(
    #     '-l', '--limb', required=True, choices=['left', 'right'],
    #     help='send joint trajectory to which limb'
    # )
    # args = parser.parse_args(rospy.myargv()[1:])
    # limb = args.limb

    # print("Initializing node... ")
    # rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    # print("Getting robot state... ")
    # rs = baxter_interface.RobotEnable(CHECK_VERSION)
    # print("Enabling robot... ")
    # rs.enable()
    # print("Running. Ctrl-c to quit")
    # positions = {
    #     'left':  [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
    #     'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
    # }

    # traj = Trajectory(limb)
    # rospy.on_shutdown(traj.stop)
    # # Command Current Joint Positions first
    # limb_interface = baxter_interface.limb.Limb(limb)
    # current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    # traj.add_point(current_angles, 0.0)

    # p1 = positions[limb]
    # traj.add_point(p1, 7.0)
    # traj.add_point([x * 0.75 for x in p1], 9.0)
    # traj.add_point([x * 1.25 for x in p1], 12.0)
    # traj.start()
    # traj.wait(15.0)
    # print("Exiting - Joint Trajectory Action Test Complete")

    rospy.init_node('baxter_smach_joint_traj_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # sm.userdata.name = []
    # sm.userdata.position = []
    
    with sm:

        smach.StateMachine.add('READ_CURRENT_JOINT_ANGLES',
                               ReadTopicState('/robot/joint_states', JointState, ['name', 'position']), 
                               transitions={'succeeded':'LEFT_LIMB_JOINT_MOTION_1'},
                               remapping={'name':'current_joint_names', 'position':'current_joint_positions'})
        
        sm.userdata.left_limb_joint_motion_1_positions = (
            [[-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
             [x * 0.75 for x in [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]],
             [x * 1.25 for x in [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]]])

        sm.userdata.left_limb_joint_motion_1_times = [7.0, 9.0, 12.0]

        def left_limb_joint_motion_1_goal_cb(userdata, goal):
            goal = FollowJointTrajectoryGoal()

            current_joint_positions = dict(zip(userdata.current_joint_names, userdata.current_joint_positions))

            current_left_joint_positions = [current_joint_positions['left_s0'],
                                            current_joint_positions['left_s1'],
                                            current_joint_positions['left_e0'],
                                            current_joint_positions['left_e1'],
                                            current_joint_positions['left_w0'],
                                            current_joint_positions['left_w1'],
                                            current_joint_positions['left_w2']]
            

            current_point = JointTrajectoryPoint(positions = copy(current_left_joint_positions),
                                                 time_from_start = rospy.Duration(0.0))
            goal.trajectory.points.append(current_point)

            for i in range(len(userdata.left_limb_joint_motion_1_positions)):
                point = JointTrajectoryPoint(positions = copy(userdata.left_limb_joint_motion_1_positions[i]),
                                             time_from_start = rospy.Duration(userdata.left_limb_joint_motion_1_times[i]))
                goal.trajectory.points.append(point)
        
            goal.trajectory.joint_names = ['left' + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

            goal.goal_time_tolerance = rospy.Time(0.1)
            goal.trajectory.header.stamp = rospy.Time.now()

            return goal

        smach.StateMachine.add('LEFT_LIMB_JOINT_MOTION_1',
                               smach_ros.SimpleActionState('/robot/limb/left/follow_joint_trajectory',
                                                            FollowJointTrajectoryAction,
                                                            goal_cb=left_limb_joint_motion_1_goal_cb,
                                                            input_keys=['current_joint_names',
                                                                        'current_joint_positions',
                                                                        'left_limb_joint_motion_1_positions',
                                                                        'left_limb_joint_motion_1_times']), 
                               transitions={'succeeded':'READ_CURRENT_JOINT_ANGLES'},
                               remapping={'current_joint_names':'current_joint_names',
                                          'current_joint_positions':'current_joint_positions',
                                          'left_limb_joint_motion_1_positions':'left_limb_joint_motion_1_positions',
                                          'left_limb_joint_motion_1_times':'left_limb_joint_motion_1_times'})
        
    sis = smach_ros.IntrospectionServer('BAXTER_SMACH_JOINT_TRAJ_TEST_SERVER', sm, '/SM_ROOT')

    sis.start()

    outcome = sm.execute()
    
    rospy.spin()

if __name__ == "__main__":
    main()
