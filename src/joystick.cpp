#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <stdio.h>
#include "Eigen/Dense"
#include <rosgraph_msgs/Clock.h>

#define PI      3.141592

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

float vel_x = 0; 
float vel_y = 0; 
float ang_z = 0; 
float vel_z = 0; 
int mode = 0;
int step = 0;

VectorXd pos_gain(3);
VectorXd ori_gain(3);

float now_pos_x;
float now_pos_y;
float now_pos_z;
VectorXd target_pos(3);

float now_ori_roll;
float now_ori_pitch;
float now_ori_yaw;
VectorXd target_ori(3);

float err_pos_x;
float err_pos_y;
float err_pos_z;
VectorXd err_pose_E(3);
VectorXd err_pose_I(3);

float err_ori_roll;
float err_ori_pitch;
float err_ori_yaw;
VectorXd err_ori(3);

MatrixXd RotMat(3, 3);
VectorXd Euler(3);
MatrixXd Rot_IE(3, 3);

geometry_msgs::Twist m_state;
std_msgs::Empty m_takeoff;
std_msgs::Empty m_land;

double Time;
double dt;
double x_circle, y_circle;
double R;

MatrixXd QuatToRotMat(float w, float x, float y, float z);
MatrixXd RotMatToEuler(MatrixXd C);
MatrixXd RotationMatrix(float yaw);

void Callback1(const sensor_msgs::Joy& msg)
{
    vel_x = (float) msg.axes[1]; //pitch
    vel_y = (float) msg.axes[0];
    ang_z = (float) msg.axes[2];
    vel_z = (float) msg.axes[3];

    if ((int) msg.buttons[8] == 1) { //mani
        mode = 1;
    }

    if ((int) msg.buttons[6] == 1) { //takeoff
        mode = 2;
    }

    if ((int) msg.buttons[7] == 1) { //land
        mode = 3;
    }
    if ((int) msg.buttons[9] == 1) { //home
        mode = 4;
    }

    if ((int) msg.buttons[10] == 1) { //trajectory
        mode = 5;
        step = 0;
    }
}

void Callback2(const gazebo_msgs::ModelStates& msg)
{
    //ROS_INFO("A\n");
    now_pos_x = (float) msg.pose[11].position.x;
    now_pos_y = (float) msg.pose[11].position.y;
    now_pos_z = (float) msg.pose[11].position.z;

    float now_quat_x = (float) msg.pose[11].orientation.x;
    float now_quat_y = (float) msg.pose[11].orientation.y;
    float now_quat_z = (float) msg.pose[11].orientation.z;
    float now_quat_w = (float) msg.pose[11].orientation.w;

    RotMat = QuatToRotMat(now_quat_w, now_quat_x, now_quat_y, now_quat_z);
    Euler = RotMatToEuler(RotMat);
    Rot_IE = RotationMatrix(Euler(2));

    now_ori_roll = Euler(0);
    now_ori_pitch = Euler(1);
    now_ori_yaw = Euler(2);

}

void Callback3(const rosgraph_msgs::Clock& msg)
{
    //Time = msg.clock.sec;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick");
    ros::NodeHandle nh;

    ros::Subscriber S_instruction = nh.subscribe("joy", 1, &Callback1);
    ros::Subscriber S_states = nh.subscribe("gazebo/model_states", 1, &Callback2);
    ros::Subscriber S_time = nh.subscribe("clock", 1, &Callback3);

    ros::Publisher P_stick_instruction = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000); //message name
    ros::Publisher P_takeoff_instruction = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1000); //message name
    ros::Publisher P_land_instruction = nh.advertise<std_msgs::Empty>("ardrone/land", 1000); //message name

    // Gain
    pos_gain << 1.5, 1.5, 1.5;
    ori_gain << 1.5, 1.5, 1.5;

    //intial Target
    target_pos << 0, 0, 0;
    target_ori << 0, 0, 0;

    dt = 0.001;
    Time = 0;
    R = 6;
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        // Calculate error
        err_pos_x = target_pos(0) - now_pos_x;
        err_pos_y = target_pos(1) - now_pos_y;
        err_pos_z = target_pos(2) - now_pos_z;
        err_ori_roll = target_ori(0) - now_ori_roll;
        err_ori_pitch = target_ori(1) - now_ori_pitch;
        err_ori_yaw = target_ori(2) - now_ori_yaw;

        // Error Transformation 
        err_pose_I << err_pos_x, err_pos_y, err_pos_z;
        err_pose_E = Rot_IE.transpose() * err_pose_I;
        err_ori << err_ori_roll, err_ori_pitch, err_ori_yaw;

        x_circle = R * cos(2 * PI * 0.5 * Time);
        y_circle = R * sin(2 * PI * 0.5 * Time);
   
        // Flight Mode 
        switch (mode) {
        case 0:
            ROS_INFO("Mode Selection!!");
            break;
        case 1:
            ROS_INFO("Control Mode");
            m_state.linear.x = 3.0 * (float) vel_x;
            m_state.linear.y = 3.0 * (float) vel_y;
            m_state.linear.z = 3.0 * (float) vel_z;
            m_state.angular.x = (float) 0;
            m_state.angular.y = (float) 0;
            m_state.angular.z = 3.0 * (float) ang_z;
            P_stick_instruction.publish(m_state);
            break;

        case 2:
            ROS_INFO("Take off");
            P_takeoff_instruction.publish(m_takeoff);
            break;

        case 3:
            ROS_INFO("Land");
            P_land_instruction.publish(m_land);
            break;

        case 4:
            ROS_INFO("Home Return!");
            if (abs(err_pose_E(0)) > 0.1 && abs(err_pose_E(1)) > 0.1) {
                target_pos << 0, 0, 8;
                m_state.linear.x = pos_gain(0) * err_pose_E(0);
                m_state.linear.y = pos_gain(1) * err_pose_E(1);
                m_state.linear.z = pos_gain(2) * err_pose_E(2);
            }
            else {
                ROS_INFO("Return Complete");
                target_pos << 0, 0, 0.5;
                m_state.linear.x = pos_gain(0) * err_pose_E(0);
                m_state.linear.y = pos_gain(1) * err_pose_E(1);
                m_state.linear.z = pos_gain(2) * err_pose_E(2);
                m_state.angular.x = ori_gain(0) * err_ori_roll;
                m_state.angular.y = ori_gain(1) * err_ori_pitch;
                m_state.angular.z = ori_gain(2) * err_ori_yaw;
            }
            P_stick_instruction.publish(m_state);
            break;
        case 5:  
            switch (step) {
            case 0:
                ROS_INFO("STEP-INIT");
                target_pos << 0, 0, 8;
                m_state.linear.x = pos_gain(0) * err_pose_E(0);
                m_state.linear.y = pos_gain(1) * err_pose_E(1);
                m_state.linear.z = pos_gain(2) * err_pose_E(2);
                if ((target_pos(0) - 0.1) < now_pos_x && now_pos_x < (target_pos(0) + 0.1) && (target_pos(1) - 0.1) < now_pos_y && now_pos_y < (target_pos(1) + 0.1) && (target_pos(2) - 0.1) < now_pos_z && now_pos_z < (target_pos(2) + 0.1)) {
                    step = 1;
                }
                break;
            case 1:
               ROS_INFO("Circle");
                target_pos << x_circle, y_circle, 8;
                target_ori << 0, 0, 0;
                m_state.linear.x = pos_gain(0) * err_pose_E(0);
                m_state.linear.y = pos_gain(1) * err_pose_E(1);
                m_state.linear.z = pos_gain(2) * err_pose_E(2);
                break;
            }
            P_stick_instruction.publish(m_state);
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
        Time = Time + dt;
    }
    return 0;
}

MatrixXd QuatToRotMat(float w, float x, float y, float z)
{
    MatrixXd tmp(3, 3);
    tmp(0, 0) = w * w + x * x - y * y - z*z;
    tmp(0, 1) = 2 * x * y - 2 * w*z;
    tmp(0, 2) = 2 * w * y + 2 * x*z;
    tmp(1, 0) = 2 * w * z + 2 * x*y;
    tmp(1, 1) = w * w - x * x + y * y - z*z;
    tmp(1, 2) = 2 * y * z - 2 * w*x;
    tmp(2, 0) = 2 * x * z - 2 * w*y;
    tmp(2, 1) = 2 * w * x + 2 * y*z;
    tmp(2, 2) = w * w - x * x - y * y + z*z;
    return tmp;
}

MatrixXd RotMatToEuler(MatrixXd C)
{
    VectorXd tmp(3);
    tmp(0) = atan2(-C(1, 2), C(2, 2));
    tmp(1) = atan2(C(0, 2), sqrt(pow(C(0, 0), 2) + pow(C(0, 1), 2)));
    tmp(2) = atan2(-C(0, 1), C(0, 0));
    return tmp;
}

MatrixXd RotationMatrix(float yaw)
{
    MatrixXd tmp(3, 3);
    tmp(0, 0) = cos(yaw);
    tmp(0, 1) = -sin(yaw);
    tmp(0, 2) = 0;
    tmp(1, 0) = sin(yaw);
    tmp(1, 1) = cos(yaw);
    tmp(1, 2) = 0;
    tmp(2, 0) = 0;
    tmp(2, 1) = 0;
    tmp(2, 2) = 1;
    return tmp;
}
