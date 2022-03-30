#!/usr/bin/env python

"""
Module containing localization interfaces for motion capture, indoor
localization, etc.
"""
from __future__ import division
import numpy as np
from threading import Thread, Event
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from svea.states import VehicleState
from svea_msgs.msg import VehicleState as VehicleStateMsg

__license__ = "MIT"
__maintainer__ = "Tobias Bolin, Frank Jiang"
__email__ = "tbolin@kth.se "
__status__ = "Development"


class LocalizationInterface(object):
    """Interface handling the reception of state information from the
    localization stack. This object can take on several callback
    functions and execute them as soon as state information is
    available.

    :param vehicle_name: Name of vehicle being controlled;
                         The name will be effectively be added as a
                         namespace to the topics used by the
                         corresponding localization node i.e
                         `namespace/vehicle_name/state`, defaults to
                         ''
    :type vehicle_name: str, optional
    """

    def __init__(self, vehicle_name=''):
        self.vehicle_name = vehicle_name
        sub_namespace = vehicle_name + '/' if vehicle_name else ''
        self._state_topic = sub_namespace + 'state'

        self.state = VehicleState()
        self.last_time = float('nan')

        self.is_ready = False
        self._ready_event = Event()
        rospy.on_shutdown(self._shutdown_callback)

        # list of functions to call whenever a new state comes in
        self.callbacks = []

    def start(self):
        """Spins up ROS background thread; must be called to start
        receiving data

        :return: itself
        :rtype: LocalizationInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _wait_until_ready(self, timeout=20.0):
        tic = rospy.get_time()
        self._ready_event.wait(timeout)
        toc = rospy.get_time()
        wait = toc - tic
        return wait < timeout

    def _shutdown_callback(self):
        self._ready_event.set()


    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Localization Interface Node for "
                      + self.vehicle_name)
        self.node_name = 'localization_node'
        self._start_listen()
        self.is_ready = self._wait_until_ready()
        if not self.is_ready:
            rospy.logwarn("Localization not responding during start of "
                          "Localization Interface. Setting ready anyway.")
        self.is_ready = True
        rospy.loginfo("{} Localization Interface successfully initialized"
                      .format(self.vehicle_name))

        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber(self._state_topic,
                         VehicleStateMsg,
                         self._read_state_msg,
                         tcp_nodelay=True,
                         queue_size=1)

    def _read_state_msg(self, msg):
        self.state.state_msg = msg
        self.last_time = rospy.get_time()
        self._ready_event.set()
        self._ready_event.clear()

        for cb in self.callbacks:
            cb(self.state)

    def add_callback(self, cb):
        """Add state callback. Every function passed into this method
        will be called whenever new state information comes in from the
        localization stack.

        :param cb: A callback function intended for responding to the
                   reception of state info
        :type cb: function
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """Remove callback so it will no longer be called when state
        information is received

        :param cb: A callback function that should be no longer used
                   in response to the reception of state info
        :type cb: function
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))

class MotionCaptureInterface(object):
    """Interface handling the reception of state information from the
    motion capture system. This object can take on several callback
    functions and execute them as soon as state information is
    available.

    :param mocap_name: Name of mocap model in Qualisys software;
                                The name will be effectively be added as a
                                namespace to the topics used by the
                                corresponding localization node i.e
                                `qualisys/model_name/odom`, defaults to
                                ''
    :type mocap_name: str, optional
    """

    def __init__(self, mocap_name=''):
        self.model_name = mocap_name
        self._mocap_topic = 'qualisys/' + self.model_name + '/odom'

        self.state = VehicleState()
        self.last_time = float('nan')

        self.is_ready = False
        self._ready_event = Event()
        rospy.on_shutdown(self._shutdown_callback)

        # list of functions to call whenever a new state comes in
        self.callbacks = []

    def start(self):
        """Spins up ROS background thread; must be called to start
        receiving data

        :return: itself
        :rtype: MotionCaptureInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _wait_until_ready(self, timeout=20.0):
        tic = rospy.get_time()
        self._ready_event.wait(timeout)
        toc = rospy.get_time()
        wait = toc - tic
        return wait < timeout

    def _shutdown_callback(self):
        self._ready_event.set()

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Motion Capture Interface Node for "
                      + self.model_name)
        self.node_name = 'motion_capture_node'
        self._start_listen()
        self.is_ready = self._wait_until_ready()
        if not self.is_ready:
            rospy.logwarn("Motion Capture not responding during start of "
                          "Motion Caputer. Setting ready anyway.")
        self.is_ready = True
        rospy.loginfo("{} Motion Capture Interface successfully initialized"
                      .format(self.model_name))

        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber(self._mocap_topic,
                         Odometry,
                         self._read_mocap_msg,
                         tcp_nodelay=True,
                         queue_size=1)

    def _read_mocap_msg(self, msg):
        self.state.odometry_msg = msg
        self.last_time = rospy.get_time()
        self._ready_event.set()
        self._ready_event.clear()

        for cb in self.callbacks:
            cb(self.state)

    def add_callback(self, cb):
        """Add state callback. Every function passed into this method
        will be called whenever new state information comes in from the
        motion capture system.

        :param cb: A callback function intended for responding to the
                   reception of state info
        :type cb: function
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """Remove callback so it will no longer be called when state
        information is received

        :param cb: A callback function that should be no longer used
                   in response to the reception of state info
        :type cb: function
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))

class LocalizationKnowledgeProvider(object):
    """Interface for providing knowledge to the localization system."""
    
    def __init__(self, svea, accuracy=0.3):
        self.svea = svea
        self.initial_pose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1
        )
        self.accuracy = accuracy
        self.is_ready = False
        
    def publish_initial_point_until_accurate(self, x, y, yaw):
        """Publish initial pose of vehicle."""
        
        pose_msg = self._get_initial_pose_message(x, y, yaw)
        self._wait_for_accuracy(pose_msg)
        
    def _get_initial_pose_message(self, x, y, yaw):
        """Get initial pose message."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        msg.pose.covariance = np.diag([0.1, 0.1, 0, 0, 0, 0.1]).flatten().tolist()
        return msg
    
    def _wait_for_accuracy(self, message):
        """Wait for accuracy."""
        rospy.loginfo("Waiting for position accuracy")
        
        error = float("inf")
        publish_rate = rospy.Rate(2)
        
        while error > self.accuracy:
            message.header.stamp = rospy.Time.now()
            self.initial_pose_pub.publish(message)
            
            current_state = self.svea.wait_for_state()
            
            error = np.sqrt((message.pose.pose.position.x - current_state.x)**2 + \
                    (message.pose.pose.position.y - current_state.y)**2)
            rospy.loginfo("Current error: {} > {}".format(error, self.accuracy))
            
            publish_rate.sleep()
            
        self.is_ready = True
        rospy.loginfo("Accuracy achieved")
        
        
localizers = {
    'indoor': LocalizationInterface,
    'mocap': MotionCaptureInterface
}
