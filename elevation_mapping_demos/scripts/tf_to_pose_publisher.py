#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros

def callback():
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""
    global publisher, tf_buffer, tf_listener, from_frame, to_frame

    # Listen to transform and throw exception if the transform is not
    # available.
    try:
        trans = tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return

    # Create and fill pose message for publishing
    pose = PoseWithCovarianceStamped()
    pose.header = trans.header
    pose.pose.pose.position.x = trans.transform.translation.x
    pose.pose.pose.position.y = trans.transform.translation.y
    pose.pose.pose.position.z = trans.transform.translation.z
    pose.pose.pose.orientation = trans.transform.rotation

    # Since tf transforms do not have a covariance, pose is filled with
    # a zero covariance.
    pose.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    publisher.publish(pose)


def main(args=None):
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    global publisher, tf_buffer, tf_listener, from_frame, to_frame

    rclpy.init(args=args)

    node = rclpy.create_node('tf_to_pose_publisher')
    
    # Read frame id's for tf listener
    node.declare_parameter("from_frame", "odom")
    node.declare_parameter("to_frame", "base_link")
    from_frame = str(node.get_parameter("from_frame").value)
    to_frame = str(node.get_parameter("to_frame").value)
    pose_name = to_frame + "_pose"

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    publisher = node.create_publisher(
        PoseWithCovarianceStamped, 
        pose_name, 
        rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

    # Set callback and start spinning
    timer = node.create_timer(0.05, callback)

    rclpy.spin(node)

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
