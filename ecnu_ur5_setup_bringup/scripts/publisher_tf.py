#!/usr/bin/env python2
"""
 Using for publish the calibrated transformation
 Without the dependence of easy hand eye package
  sunbin 2017.8.23

"""
import rospy
import yaml
from tf import TransformBroadcaster, TransformerROS, transformations as tfs
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

inverse = False

# remove the dependence of easy hand eye
# the eye on hand config filename
filename = "/home/sun/ws/ecnu_ur5_setup_ws/src/ecnu_ur5_setup/ecnu_ur5_setup_bringup/config/easy_handeye_eye_on_hand.yaml"
# the eye to hand config filename
#filename = "/home/sun/ws/ecnu_ur5_setup_ws/src/ecnu_ur5_setup/ecnu_ur5_setup_bringup/config/easy_handeye_eye_on_base.yaml"

# global var
robot_effector_frame = ""
robot_base_frame = "base_link"
tracking_base_frame = "camera_link"
eye_on_hand = None
transformation = TransformStamped()

def from_dict(in_dict):
    """
    Sets values parsed from a given dictionary.

    :param in_dict: input dictionary.
    :type in_dict: dict[string, string|dict[string,float]]

    :rtype: None
    """
    global robot_base_frame
    global robot_effector_frame
    global tracking_base_frame
    global eye_on_hand
    global transformation
    eye_on_hand = in_dict['eye_on_hand']
    robot_base_frame = in_dict['robot_base_frame']
    tracking_base_frame = in_dict['tracking_base_frame']
    transformation = TransformStamped(
        child_frame_id=in_dict['tracking_base_frame'],
        transform=Transform(
            Vector3(in_dict['transformation']['x'],
                    in_dict['transformation']['y'],
                    in_dict['transformation']['z']),
            Quaternion(in_dict['transformation']['qx'],
                       in_dict['transformation']['qy'],
                       in_dict['transformation']['qz'],
                       in_dict['transformation']['qw'])
        )
    )
    if eye_on_hand:
        robot_effector_frame = in_dict['robot_effector_frame']
        transformation.header.frame_id = in_dict['robot_effector_frame']
    else:
        transformation.header.frame_id = in_dict['robot_base_frame']

def from_yaml(in_yaml):
    """
    Parses a yaml string and sets the contained values in this calibration.

    :param in_yaml: a yaml string
    :rtype: None
    """
    from_dict(yaml.load(in_yaml))

with open(filename) as calib_file:
    from_yaml(calib_file.read())




rospy.loginfo('loading calibration parameters ')


orig = transformation.header.frame_id  # tool or base link
dest = transformation.child_frame_id  # tracking_base_frame

transformer = TransformerROS()
result_tf = transformation.transform
transl = result_tf.translation.x, result_tf.translation.y, result_tf.translation.z
rot = result_tf.rotation.x, result_tf.rotation.y, result_tf.rotation.z, result_tf.rotation.w
cal_mat = transformer.fromTranslationRotation(transl, rot)
if inverse:
    cal_mat = tfs.inverse_matrix(cal_mat)
    orig, dest = dest, orig
translation = tfs.translation_from_matrix(cal_mat)
rotation = tfs.quaternion_from_matrix(cal_mat)

rospy.loginfo('publishing transformation ' + orig + ' -> ' + dest + ':\n' + str((translation, rotation)))

broad = TransformBroadcaster()

rate = rospy.Rate(50)

while not rospy.is_shutdown():
    broad.sendTransform(translation, rotation, rospy.Time.now(), dest, orig)  # takes ..., child, parent
    rate.sleep()
