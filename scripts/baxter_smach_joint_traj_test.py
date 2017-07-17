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

import baxter_interface

from baxter_interface import CHECK_VERSION

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


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


# class ReadTopicState(WaitForMsgState):
#     def __init__(self, topic, msg_type, fields, **kwargs):
#         self._fields = fields
#         WaitForMsgState.__init__(self, topic, msg_type, msg_cb=self._msg_cb, latch=True, output_keys=fields, **kwargs)
# 
#     def _msg_cb(self, msg, userdata):
#         for field in self._fields:
#             field_reader = rostopic.msgevalgen(field)
#             setattr(userdata, field, field_reader(msg))
# 
#         return msg is not None


class ReadLimbJointsState(WaitForMsgState):
    def __init__(self, limb, topic='/robot/joint_states', **kwargs):
        self._limb = limb
        
        # Use the super class to read from topic
        super(ReadLimbJointsState, self).__init__(topic, JointState, msg_cb=self._msg_cb, latch=True, output_keys=['joint_positions'], **kwargs)

    def _msg_cb(self, msg, userdata):
        # Read 'name' and 'position' fields from topic message
        name_field_reader = rostopic.msgevalgen('name')
        joint_names = name_field_reader(msg)
        position_field_reader = rostopic.msgevalgen('position')
        joint_positions = position_field_reader(msg)
        
        # Create a dict of names/positions
        current_joint_positions = dict(zip(joint_names, joint_positions))

        # Create a userdata joint_positions entry using the joint positions for the selected limb
        userdata['joint_positions'] = [current_joint_positions[self._limb + '_s0'],
                                       current_joint_positions[self._limb + '_s1'],
                                       current_joint_positions[self._limb + '_e0'],
                                       current_joint_positions[self._limb + '_e1'],
                                       current_joint_positions[self._limb + '_w0'],
                                       current_joint_positions[self._limb + '_w1'],
                                       current_joint_positions[self._limb + '_w2']]

        return msg is not None


class FollowJointTrajectoryActionState(smach.State):
    def __init__(self, traj_client):
        smach.State.__init__(self, 
                             outcomes=['succeeded'],
                             input_keys=['current_joint_positions',
                                         'via_points', 'via_times',
                                         'target_joint_positions', 'target_time'])

        # Save reference to trajectory client object
        self._traj_client = traj_client
        rospy.on_shutdown(self._traj_client.stop)

    def execute(self, userdata):
        # Add current joint positions to trajectory
        # (presumed to have been read into userdata from previous state, e.g. ReadLimbJointsState)
        self._traj_client.add_point(userdata.current_joint_positions, 0.0)

        # Add via points and times to trajectory
        for point, time in zip(userdata.points, userdata.times):
            self._traj_client.add_point(point, time)

        # Start motion
        self._traj_client.start()

        # Wait for result from action client (important!)
        self._traj_client.wait(15.0)

        return 'succeeded'


class UserDataToOutcomeState(smach.State):
    """This state returns the userdata value for given input_key as its outcome.
    """
    def __init__(self, outcomes, input_key, ud_to_value_func):
        outcomes = outcomes[:] # copy needed to extend list
        outcomes.extend(['field_error', 'undefined_outcome'])
        smach.State.__init__(self, outcomes, [input_key])
        self._outcomes = outcomes
        self._input_key = input_key
#        self._field = 'task_id'
        self._ud_to_value_func = ud_to_value_func

    def execute(self, ud):
        field = self._ud_to_value_func(ud[self._input_key])
#        field = ud[self._input_key][self._field]
        print "got field as: ", field
        if field is None:
            return 'field_error'
        elif field in self._outcomes:
            return field
        else:
            return 'undefined_outcome'


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
    
    print("Starting Baxter SMACH Joint Trajectory Action Test.")

    print("Initializing node...")
    rospy.init_node('baxter_smach_joint_traj_test')
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    print("Initializing follow joint trajectory action clients for each limb... ")
    left_traj_client = Trajectory('left')
    right_traj_client = Trajectory('right')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:

        # smach.StateMachine.add('READ_CURRENT_JOINT_ANGLES',
        #                        ReadTopicState('/robot/joint_states', JointState, ['name', 'position']),
        #                        transitions={'succeeded':'LEFT_LIMB_JOINT_MOTION_1'},
        #                        remapping={'name':'current_joint_names', 'position':'current_joint_positions'})
        
        smach.StateMachine.add('READ_LEFT_LIMB_JOINTS_1',
                               ReadLimbJointsState('left'),
                               remapping={'joint_positions':'left_limb_joint_positions_1'},
                               transitions={'succeeded':'GENERATE_LEFT_LIMB_OUTWARD_TRAJ'})
        
        sm.userdata.left_limb_outward_traj_points = (
            [[-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
             [x * 0.75 for x in [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]],
             [x * 1.25 for x in [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]]])
        
        sm.userdata.left_limb_outward_traj_times = [0.0, 7.0, 9.0, 12.0]
        
        StateMachine.add('GENERATE_LEFT_LIMB_OUTWARD_TRAJ',
                         UserdataToOutputState(lambda ud: [ud.left_limb_joint_positions_1] + ud.left_limb_outward_traj_points),
                         remapping={'output':'left_limb_outward_traj_points'},
                         transitions={'succeeded':'LEFT_LIMB_JOINT_MOTION_1')

        smach.StateMachine.add('LEFT_LIMB_JOINT_MOTION_1',
                               FollowJointTrajectoryActionState(left_traj_client),
                               remapping={'points':'left_limb_outward_traj_points', 'times':'left_limb_outward_traj_times'}
                               transitions={'succeeded':'succeeded'})

        # smach.StateMachine.add('READ_LEFT_LIMB_JOINTS_2',
        #                        ReadLimbJointsState('left'),
        #                        remapping={'joint_positions':'left_limb_joint_positions_2'},
        #                        transitions={'succeeded':'GENERATE_LEFT_LIMB_RETURN_TRAJ'})

        # StateMachine.add('GENERATE_LEFT_LIMB_RETURN_TRAJ',
        #                  UserdataToOutputState(lambda ud: [ud.left_limb_joint_positions_2] + [ud.left_limb_joint_motion_1_points ),
        #                  remapping={'output':'left_limb_return_traj'},
        #                  transitions={'succeeded':'succeeded')
        
        # smach.StateMachine.add('READ_LEFT_LIMB_JOINTS_2',
        #                        ReadLimbJointsState('left'),
        #                        remapping={'joint_positions':'left_limb_joint_positions_2'},
        #                        transitions={'succeeded':'LEFT_LIMB_JOINT_MOTION_2'})
        # 
        # sm.userdata.left_limb_joint_motion_2_points = (
        #     [[x * 0.75 for x in [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
        #      [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
        #      sm.userdata.left_limb_joint_positions_1]])

        # sm.userdata.left_limb_joint_motion_2_times = [7.0, 9.0, 12.0]
        # 
        # smach.StateMachine.add('LEFT_LIMB_JOINT_MOTION_2',
        #                        FollowJointTrajectoryActionState(left_traj_client),
        #                        remapping={'current_joint_positions':'left_limb_joint_positions_2',
        #                                   'points':'left_limb_joint_motion_2_points',
        #                                   'times':'left_limb_joint_motion_2_times'},
        #                        transitions={'succeeded':'succeeded'})
        
    sis = smach_ros.IntrospectionServer('BAXTER_SMACH_JOINT_TRAJ_TEST_SERVER', sm, '/SM_ROOT')

    sis.start()

    outcome = sm.execute()
    
    print("Baxter SMACH Joint Trajectory Action Test Complete. Ctrl-C to exit.")
    
    rospy.spin()

if __name__ == "__main__":
    main()
