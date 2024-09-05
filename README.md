# Turtlebot3 simulation

![](https://github.com/user-attachments/assets/d2a78e83-38af-47b7-9473-057e7c0d8ed6)


## Project description:
1. Simulating Turtlebot3 using Gazebo.
2. LiDAR and IMU Sensor Data Visualization in RViz.
3. Unit Conversion Node Implementation in C++.
4. Uni-dimensional Kalman filter node implemented in python.
5. Testing filtered data in comparison with raw data using rqt_multiplot.
   
---
## Deploying Turtlebot 3:
For deploying Turtlebot3, follow Turtlebot3 gazebo [instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation). In our example, we used the Turtlebot3 world.
![image](https://github.com/user-attachments/assets/1207fdf7-01b3-4c8c-9995-374ab262f13e)
In the root directory of the workspace:
```bash
catkin_make
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
To control the robot using the keyboard, in a new terminal window:
```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
---
## Visualizing LiDar Data on RVIZ
![Screenshot from 2024-09-05 03-28-16](https://github.com/user-attachments/assets/6ac0bb56-b3e1-4219-8f03-e8b524fcfbfa)

On a new terminal window:
```bash
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

## Unit conversion node
Quaternion is a four-dimensional vector used to represent object orientation in 3D space.

$$
q=w+xi+yj+zk
$$

The conversion to Euler's RPY can be done on several steps: 
1- Conversionn to rotation matrix:

$$
R=
\begin{bmatrix}
​1−2(y2+z2) & 2(xy+wz) & 2(xz−wy)​ \cr
2(xy−wz) & 1−2(x2+z2) & 2(yz+wx) \cr
​2(xz+wy) & 2(yz−wx) & 1−2(x2+y2)​
\end{bmatrix}
$$

From the rotation Matrix:

$$
 \text{Yaw} = \text{atan2}(R_{21}, R_{11}) \
$$

$$
\text{Pitch} = \text{atan2}\left(-R_{31}, \sqrt{R_{32}^2 + R_{33}^2}\right) 
$$

$$
\text{Roll} = \text{atan2}(R_{32}, R_{33})
$$

Luckily, this can be done by using the tf2 library, which provides a Matrix3x3 function which calculates the rotation matrix, and getRPY function to extract the Euler Angles.
The library provides other similar functions, so to choose carefully we first needed to check the library's documentation and [source code](https://github.com/ros/geometry2/tree/noetic-devel/tf2/include/tf2/LinearMath)

The unit conversion node is based on the OOP principles, following pure i/o functions, each class has its own precise purpose and deals with specific type of data.



AxisRot class, responsible for Unit conversion,
```Cpp
struct AxisRot {
    double yaw =0, pitch=0, roll=0;
    AxisRot() {};
    AxisRot(double yaw, double pitch, double roll): yaw(yaw), pitch(pitch), roll(roll) {}
    AxisRot(tf2::Quaternion q) {
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll, pitch, yaw);
    }
};
```

FetchQuaternion: contains the call back function, fetches data and stores it appropriate variable.
```Cpp
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
```

The call back functions cannot return a type. To solve this issue, we can use an object method to store data:
```Cpp
    ros::Subscriber sub = nh.subscribe("/imu", 1000, &FetchQuaternion::fetch_data, &fq);
```

---
## Kalman filter
Base terms for Kalman filter:
- *Process variance:* How much uncertainty in the system.
- *Measurement variance:* How much uncertainty in the measurements.
- *Initail estimate:* A guess for the value being estimated.
- *Initial error estimate:* A guess for how uncertain the that initial estimate is.


Uni dimensional Kalman filter is based on the following equation:


$$
P_{k∣k−1}​=P_{k−1∣k−1}​+Q
$$

- $P_{k∣k−1​}$: predicted estimate covariance (error estimate) at time kk.
- $P_{k−1∣k−1}$: the updated error estimate from the previous step.
- $Q$: process variance.
  
This can be eaily implemented:
```Python
def predict(self):
    self.error_estimate += self.process_variance
```
The Kalman gain determines how much the new measurement should adjust the estimate. It balances the trust between the prediction and the measurement.

$$
K_k = \frac{P_{K|k-1}}{P_{K|k-1}+R}
$$

- $R$: measurement variance.

In our code, this translates to:
```Python
def update(self, measurement):
    kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
    self.estimate += kalman_gain * (measurement - self.estimate)
    self.error_estimate *= (1 - kalman_gain)

```

Updated estimation:

$$
X_{k|k} = X_{k|k-1} + K_k * (Z_k - X_{k|k-1})
$$

- $X_{k|k}$: The updated state estimate after incorporating the new measurement.
- $X_{k|k-1}$: The predicted state estimate.
- $Z_k$: The new measurement.
- $K_k$: The Kalman gain.

```Python
self.estimate += kalman_gain * (measurement - self.estimate)
```

Updating the error estimate:

$$
P_{k∣k​}=(1−K_k​)P_{k∣k−1}
$$
  
```Python
  self.error_estimate *= (1 - kalman_gain)
```
* The node reads the Yaw value from the */imu_degree* topic (Float64MultiArray) and republish it on */filtered_yaw* topic (Float64)
  
---
## Testing the kalman Filter:
After [installing](https://github.com/ANYbotics/rqt_multiplot_plugin/blob/master/README.md) rqt_multiplot and dependenies, on a new terminal window:

### Setup rqt_multiplot:

```Bash
rosrun rqt_multiplot rqt_multiplot
```

### Visualize the curves:
In the src directory:
```bash
mkdir imu_sensor data $$ cd imu_sensor_data
git clone https://github.com/minawashere/MIA-Team-8-task-10.1 #clone our repo
cd ~/catkin_ws
catkin_make
```
Launch the converter and the filtering nodes: 
```Bash
rosrun imu_sensor_data quaternion_to_degree & #to run it in the backgroung
rosrun imu_sensor_data kalman_filter.py &
```
To add noise to the gazebo environment, we have to check the configuration files of turtlebot3. To be sure of what value we are changing, we will check their source [code](https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro)
, in [line 85](https://github.com/ROBOTIS-GIT/turtlebot3/blob/66681b33749c44e7d9022253ac210ef2da7843a0/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro#L85C42-L85C62), it states that the imu plugin for our model is *libgazebo_ros_imu.so*. Googling it, we find it's [source code](https://docs.ros.org/en/melodic/api/gazebo_plugins/html/gazebo__ros__imu_8cpp_source.html)
scroll down:
![image](https://github.com/user-attachments/assets/9ce013cc-ee19-4cc5-bf31-6e41810aa591)

```xacro
<gaussianNoise>0.0</gaussianNoise> #this is the value we are looking to change
```

To add noise to our moodel:

```bash
cd /opt/ros/noetic/share/turtlebot3_description/urdf && nvim turtlebot3_waffle.gazebo.xacro
````
scroll down to the imu_plugin configuration, and change the gaussian noise value to <0.5>

Open rqt_multiplot tab, On the top right part click `settings > add new curve`. On the *x-axis* check the time receipt, on the y-axis choode the */imu_degree topic*, make sure you are printing
the *data/0* field.
repeat the same steps for the */filtered_yaw* topic.

![image](https://github.com/user-attachments/assets/0590bd64-1426-4e33-8a3f-2ac13411b6f2)






