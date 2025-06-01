import rospy
import tf2_ros
import geometry_msgs.msg
import math
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler

# Global variables to store last two JointState messages
last_msg = None
current_msg = None

def rotating_base_link_callback(msg):
    global last_msg, current_msg

    # Shift current_msg to last_msg and update current_msg
    last_msg = current_msg
    current_msg = msg
    print(current_msg)


def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quat_static =  quaternion_from_euler(r , p, h)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = quat_static[0]
    t.transform.rotation.y = quat_static[1]
    t.transform.rotation.z = quat_static[2]
    t.transform.rotation.w = quat_static[3]
    #print(t)
    br.sendTransform(t)



def interpolate_and_publish():
    global last_msg, current_msg

    if last_msg is None or current_msg is None:
        return  # Not enough data to interpolate

    # Get the current time
    current_time = rospy.Time.now()

    # Interpolate position based on the time difference
    time_diff = (current_msg.header.stamp - last_msg.header.stamp).to_sec()
    if time_diff <= 0:
        return  # Avoid division by zero or negative time differences

    alpha = (current_time - last_msg.header.stamp).to_sec() / time_diff
    interpolated_position = last_msg.position[0] + alpha * (current_msg.position[0] - last_msg.position[0])

    # Create and publish tf for Hokuyo
    q_hokoyo = quaternion_from_euler(90 * math.pi / 180, -90 * math.pi / 180, interpolated_position * math.pi / 180)
    br_hokydo = tf2_ros.TransformBroadcaster()
    t_hokoyo = geometry_msgs.msg.TransformStamped()
    t_hokoyo.header.stamp = current_time
    t_hokoyo.header.frame_id = "rotation_base"
    t_hokoyo.child_frame_id = "laser"
    t_hokoyo.transform.translation.x = 0.00
    t_hokoyo.transform.translation.y = 0.06
    t_hokoyo.transform.translation.z = 0.06
    t_hokoyo.transform.rotation.x = q_hokoyo[0]
    t_hokoyo.transform.rotation.y = q_hokoyo[1]
    t_hokoyo.transform.rotation.z = q_hokoyo[2]
    t_hokoyo.transform.rotation.w = q_hokoyo[3]
    br_hokydo.sendTransform(t_hokoyo)

    # Create and publish tf for Velodyne
    q_velodyne = quaternion_from_euler(-90 * math.pi / 180, 0 * math.pi / 180, interpolated_position * math.pi / 180)
    br_velodune = tf2_ros.TransformBroadcaster()
    t_velodyne = geometry_msgs.msg.TransformStamped()
    t_velodyne.header.stamp = current_time
    t_velodyne.header.frame_id = "rotation_base"
    t_velodyne.child_frame_id = "velodyne"
    t_velodyne.transform.translation.x = 0.00
    t_velodyne.transform.translation.y = -0.04
    t_velodyne.transform.translation.z = 0.06
    t_velodyne.transform.rotation.x = q_velodyne[0]
    t_velodyne.transform.rotation.y = q_velodyne[1]
    t_velodyne.transform.rotation.z = q_velodyne[2]
    t_velodyne.transform.rotation.w = q_velodyne[3]
    br_velodune.sendTransform(t_velodyne)


    # static_transform_publisher("camera_pose_frame", "rotation_base",  0, 0, 0,    0*math.pi/180, 0*math.pi/180, 90*math.pi/180)
    # static_transform_publisher("camera_init", "camera_odom_frame",  0.0, 0.0, 0.0,    0*math.pi/180, 0*math.pi/180, 0*math.pi/180)
    static_transform_publisher("camera_init", "rotation_base",  0.0, 0.0, 0.0,    0*math.pi/180, 0*math.pi/180, 0*math.pi/180)

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_by_real_sense_imu')
    
    rospy.Subscriber("/orion_rotating_base/joint_states", JointState, rotating_base_link_callback, queue_size=5)

    # Set the rate to 100 Hz
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        interpolate_and_publish()
        rate.sleep()

    rospy.spin()
