#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float64

class KalmanFilter1D:
    def __init__(self, process_variance, measurement_variance, initial_estimate, initial_error_estimate):
        self.process_variance = process_variance    
        self.measurement_variance = measurement_variance
        self.estimate = initial_estimate                  
        self.error_estimate = initial_error_estimate      

    def predict(self):
        self.error_estimate += self.process_variance  

    def update(self, measurement):
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.error_estimate *= (1 - kalman_gain)

    def get_estimate(self):
        return self.estimate

def euler_callback(data):
    if len(data.data) > 0:
        yaw_measurement = data.data[0]
        kalman_filter.predict()
        kalman_filter.update(yaw_measurement)
        filtered_yaw = kalman_filter.get_estimate()
        
        filtered_yaw_pub.publish(filtered_yaw)

if __name__ == '__main__':
    rospy.init_node('kalman_filter_1d')

    process_variance = rospy.get_param('~process_variance', 0.05)
    measurement_variance = rospy.get_param('~measurement_variance', 0.09)
    initial_yaw_angle = rospy.get_param('~initial_yaw_angle', 0.0)
    initial_error_estimate = rospy.get_param('~initial_error_estimate', 1.0)

    global kalman_filter
    kalman_filter = KalmanFilter1D(process_variance, measurement_variance, initial_yaw_angle, initial_error_estimate)
    
    rospy.Subscriber('/imu_degree', Float64MultiArray, euler_callback)
    global filtered_yaw_pub
    filtered_yaw_pub = rospy.Publisher('/filtered_yaw', Float64, queue_size=10)

    rospy.spin()
