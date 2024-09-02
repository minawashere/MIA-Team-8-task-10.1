
import rospy
import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def call_back(data: Imu):
    roll, pitch, yaw =  tf.transformations.euler_from_quaternion((
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    ))
    rospy.loginfo(f" \nConverted Values:  \nRoll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
    

def publish():
    pass

def main():
    rospy.init_node("imu_readings_in_degree")
    rospy.Subscriber("/imu", Imu, call_back)
    
    pub = rospy.Publisher('/imu_in_degree', Float64) 
    pub.publish()

    rospy.spin()

if __name__ == "__main__":
    main()