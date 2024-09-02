#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


struct AxisRot {
    double yaw =0, pitch=0, roll=0;
    AxisRot() {};
    AxisRot(double yaw, double pitch, double roll): yaw(yaw), pitch(pitch), roll(roll) {}
    AxisRot(tf2::Quaternion q) {
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll, pitch, yaw);
    }
};


class FetchQuaternion {
    tf2::Quaternion q;

public:
    tf2::Quaternion get() {
        return q;
    }

    void fetch_data(const sensor_msgs::Imu::ConstPtr& msg) {
        q.setX(msg->orientation.x);
        q.setY(msg->orientation.y);
        q.setZ(msg->orientation.z);
        q.setW(msg->orientation.w);
    }

};


void publish_rot(AxisRot rot, ros::Publisher pub) {
    std_msgs::Float64MultiArray msg;

    msg.data.resize(3);
    msg.data[0] = rot.yaw;
    msg.data[1] = rot.pitch;
    msg.data[2] = rot.roll;

    pub.publish(msg);
    ROS_INFO("/nYaw: %f\nPitch: %f\nRoll: %f\n", msg.data[0], msg.data[1], msg.data[2]);
}

int main(int argc, char **argv) {
    FetchQuaternion fq;
    

    ros::init(argc, argv, "quaternion_to_degree");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu", 1000, &FetchQuaternion::fetch_data, &fq);
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/imu_in_degree", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        publish_rot(AxisRot(fq.get()), pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
