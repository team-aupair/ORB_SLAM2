#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import time
import sys
import math

# load pepper_io module
import pepper_io
import pepper_config

import rospy
from tf import transformations
from tf import TransformBroadcaster
import numpy

params = pepper_config.load_config()
pio = pepper_io.pepper_io(params)

def get_distance(p_f, p_i):
    return math.sqrt((p_f[0]-p_i[0]) ** 2 + (p_f[1]-p_i[1]) ** 2 + (p_f[2]-p_i[2]) ** 2)

def measure_scale():
    measure_num = 3
    scale = 0.0

    for i in range(measure_num):
        pio.say("Please move me to proper starting point.")
        pio.activate_keyboard_control()
        pio.say("Measurement " + str(i) + " \\pau=250\\")
        try:
            (trans_pi,_) = pio.transform.lookupTransform('/map', '/CameraTop_optical_frame',  rospy.Time(0))
            (trans_oi,_) = pio.transform.lookupTransform('/orb_map', '/orb_pose', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pio.say("Error detected at lookup initial transform. \\pau=100\\ Aborting.")
            return -1

        pio.activate_keyboard_control()
    
        try:
            (trans_pf,_) = pio.transform.lookupTransform('/map', '/CameraTop_optical_frame', rospy.Time(0))
            (trans_of,_) = pio.transform.lookupTransform('/orb_map', '/orb_pose', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pio.say("Error detected at lookup final transform. \\pau=100\\ Aborting.")
            return -1

        distance = get_distance(trans_pf, trans_pi) / get_distance(trans_of, trans_oi)
        print("measurement "+str(i)+": "+str(distance))
        scale += distance

    scale /= measure_num
    return scale
    
def set_scale(scale):
    if rospy.has_param('orb_scale'):
        rospy.set_param('orb_scale', scale)
        return True
    return False

def set_matrix(translation, quaternion):
    if rospy.has_param('orb_translation') and rospy.has_param('orb_quaternion'):
        rospy.set_param('orb_translation', translation.tolist())
        rospy.set_param('orb_quaternion', quaternion.tolist())    
        return True
    return False

def get_map_transform():
    try:
        (trans_map,rot_map) = pio.transform.lookupTransform('/map', '/CameraTop_optical_frame', rospy.Time(0))
        (trans_orb,rot_orb) = pio.transform.lookupTransform('/orb_pose', '/orb_map', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pio.say("Error detected at lookup current transform. \\pau=100\\ Aborting.")
        return None, None

    t_MapToCameraTop = transformations.concatenate_matrices(transformations.translation_matrix(trans_map), transformations.quaternion_matrix(rot_map))
    t_PoseToOrbMap = transformations.concatenate_matrices(transformations.translation_matrix(trans_orb), transformations.quaternion_matrix(rot_orb))
    t_MapToOrbMap = numpy.matmul(t_MapToCameraTop, t_PoseToOrbMap)
    
    translation = transformations.translation_from_matrix(t_MapToOrbMap)
    quaternion = transformations.quaternion_from_matrix(t_MapToOrbMap)

    print("# TRANSLATION: " + str(translation))
    print("# QUATERNION: " + str(quaternion))
    return translation, quaternion


pio.set_volume(1.0)
pio.say('Starting coordinate settings for O.R.B SLAM 2. \\pau=250\\ Please move me and set the location correctly.')
pio.activate_keyboard_control()

pio.say("This script is for making new O.R.B SLAM map. \\pau=150\\  Please remove the map in the path and execute O.R.B SLAM. \\pau=200\\ please input any key after executing O.R.B SLAM 2 is done. ")
raw_input("input any key when ready: ");
set_scale(1.0)

translation, quaternion = get_map_transform()

if not ((translation is None) or (quaternion is None)):
    set_matrix(translation, quaternion)
    pio.say("Please build the map, and exit when finished. \\pau=250\\")

    pio.activate_keyboard_control()

    pio.say("Please prepare the both SLAM systems properly, \\pau=100\\ and move me straight to measure the scale difference \\pau=250\\")
    scale = measure_scale()
    if (scale > 0 and set_scale(scale)):
        pio.say("Measured scale is \\pau=250\\ " + str(scale))

        print("# SCALE: " + str(scale))
        print("# TRANSLATION: " + str(translation))
        print("# QUATERNION: " + str(quaternion))
        print("#####################################\nSave this information to your ORB settings file!")
        print("(default path: $(path_to_ORB_SLAM2)/Examples/RGB-D/TUM_pepper.yaml)")
