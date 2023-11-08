#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm> // std::min_element 사용을 위한 헤더
#include <iostream>
#include <cmath>

using namespace std;

bool lidar_left = false;
bool lidar_right = false;
bool lidar_center = false;
bool lidar_endright = false;
bool lidar_endleft = false;

struct Quaternion {
    double x, y, z, w;
};

Quaternion eulerToQuaternion(double yaw) {
    Quaternion quat;

    double halfYaw = yaw * 0.5;
    double cosYaw = cos(halfYaw);
    double sinYaw = sin(halfYaw);

    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = sinYaw;
    quat.w = cosYaw;

    return quat;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg)
{
    // 왼쪽 라이다
    double min_left_angle =  0.2617993878;// 최소 각도
    double max_left_angle = 1.0471975512;  // 최대 각도

    int min_left_index = (min_left_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;
    int max_left_index = (max_left_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;

    double min_left_distance = lidar_msg->ranges[min_left_index];
    for (int i = min_left_index + 1; i <= max_left_index; ++i)
    {
        if (lidar_msg->ranges[i] < min_left_distance)
        {
            min_left_distance = lidar_msg->ranges[i];
        }
    }

    if (min_left_distance < 1.6)
    {
        std::cout << "Left Lidar detects obstacle." << std::endl;
        lidar_left = true; // 왼쪽 라이다 감지 여부 업데이트
    }
    else
    {
        lidar_left = false; // 왼쪽 라이다 감지 여부 초기화
    }



    // 오른쪽 라이다
    double min_right_angle = -0.2617993878; // 최소 각도
    double max_right_angle = -1.0471975512;  // 최대 각도

    int min_right_index = (min_right_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;
    int max_right_index = (max_right_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;

    double min_right_distance = lidar_msg->ranges[min_right_index];
    for (int i = min_right_index + 1; i <= max_right_index; ++i)
    {
        if (lidar_msg->ranges[i] < min_right_distance)
        {
            min_right_distance = lidar_msg->ranges[i];
        }
    }

    if (min_right_distance < 1.6)
    {
        std::cout << "Right Lidar detects obstacle." << std::endl;
        lidar_right = true; // 오른쪽 라이다 감지 여부 업데이트
    }
    else
    {
        lidar_right = false; // 오른쪽 라이다 감지 여부 초기화
    }


    // end right lidar
    double min_endright_angle = 1.02;
    double max_endright_angle = 1.04;

    int min_endright_index = (min_endright_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;
    int max_endright_index = (max_endright_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;

    double min_endright_distance = lidar_msg->ranges[min_endright_index];
    for (int i = min_endright_index + 1; i <= max_endright_index; ++i)
    {
        if (lidar_msg->ranges[i]< min_endright_distance)
        {
            min_endright_distance = lidar_msg->ranges[i];
        }
    }

    if(min_endright_distance < 2.0)
    {
        //std::cout << "end right detect" << std::endl;
        lidar_endright = true;
    }
    else
    {
        lidar_endright = false;
    }

    // end left lidar
    double min_endleft_angle = -1.05;
    double max_endleft_angle = -1.03;

    int min_endleft_index = (min_endleft_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;
    int max_endleft_index = (max_endleft_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;

    double min_endleft_distance = lidar_msg->ranges[min_endleft_index];
    for (int i = min_endleft_index + 1; i <= max_endleft_index; ++i)
    {
        if (lidar_msg->ranges[i]< min_endleft_distance)
        {
            min_endleft_distance = lidar_msg->ranges[i];
        }
    }

    if(min_endleft_distance < 2.0)
    {
        //std::cout << "end left detect" << std::endl;
        lidar_endleft = true;
    }
    else
    {
        lidar_endleft = false;
    }


    // 중앙 라이다
    double min_center_angle = -0.2443460953; // 최소 각도
    double max_center_angle = 0.2443460953;  // 최대 각도

    int min_center_index = (min_center_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;
    int max_center_index = (max_center_angle - lidar_msg->angle_min) / lidar_msg->angle_increment;

    double min_center_distance = lidar_msg->ranges[min_center_index];
    for (int i = min_center_index + 1; i <= max_center_index; ++i)
    {
        if (lidar_msg->ranges[i] < min_center_distance)
        {
            min_center_distance = lidar_msg->ranges[i];
        }
    }    

    if (min_center_distance < 1.8)
    {
        std::cout << "Center Lidar detects obstacle." << std::endl;
        lidar_center = true; // 중앙 라이다 감지 여부 업데이트
    }
    else
    {
        lidar_center = false; // 중앙 라이다 감지 여부 초기화
    }



}



geometry_msgs::PoseStamped current_pose;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setpoint_publisher");
    ros::NodeHandle nh;

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 20);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 20);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);

    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidarCallback);

    ros::Rate rate(20);

    // 사용자 입력을 받는 부분
    int userInput;
    std::cout << "Enter 1 to target position  , 2 to HOME position : ";
    std::cin >> userInput;

    switch(userInput) {
        case 1: {
            // 기존 코드를 여기에 삽입합니다...
                double origin_x=0, origin_y=0;

                /**/std::cout << "Origin locatioin input (base position is new terminel input cmd (rostopic echo /mavros/local_position/pose)), (x , y):" ;
                std::cin >> origin_x >> origin_y;


                double yaw_degrees;
                yaw_degrees = fmod(yaw_degrees, 360);

                double tt = 90;
                std::cout << "eular degrees (yaw) input : ";
                std::cin >> yaw_degrees;
                
                double result_degrees = -yaw_degrees + tt;
                double yaw_radians = result_degrees * M_PI / 180.0; // 라디안으로 변환

                Quaternion quat = eulerToQuaternion(yaw_radians);

                double opposite = fmod(yaw_degrees + 180, 360); // 반대 방향 계산


                std::cout << "ENU quaternion: ";
                std::cout << "x: " << quat.x << ", y: " << quat.y << ", z: " << quat.z << ", w: " << quat.w << std::endl;
                std::cout << "Opposite : degrees + 180 = "<< opposite << std::endl;;

                double position_x=0, position_y=0, position_z=3;
                double distance = 0;
                double land_z;

                double velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0; // 초기값 설정

                std::cout << "Enter distance(m): ";
                std::cin >> distance;

                std::cout << "Enter position z(1.6m<z): ";
                std::cin >> position_z;

                std::cout << "LAND position z(1.2m>z): ";
                std::cin >> land_z;

                double ddx = result_degrees * M_PI / 180.0; 
                double ddy = result_degrees * M_PI / 180.0;

                double x_l = distance * cos(ddx);
                double y_l = distance * sin(ddy);



                ////////////////////////////////////////
                //     원점값을 목표값으로 설정하는 변수     //
                double x_ll = origin_x + x_l;         //
                double y_ll = origin_y + y_l;         //
                ////////////////////////////////////////

                ////////////////////////////////////////

                double a_total = 120 + result_degrees;

                double angle_s = sin(a_total * M_PI / 180.0);
                double angle_c = cos(a_total * M_PI / 180.0);

                double v_x = 15 * angle_c;
                double v_y = 15 * angle_s;

                /////////////////////////////////////////

                double reversea_total = -120 + result_degrees;

                double angle_rs = sin(reversea_total * M_PI / 180.0);
                double angle_rc = cos(reversea_total * M_PI / 180.0);

                double rv_x = 15 * angle_rc;
                double rv_y = 15 * angle_rs;

                ////////////////////////////////////

                double center_total = 180 + result_degrees;

                double  angle_cs = sin(center_total * M_PI / 180.0);
                double  angle_cc = cos(center_total * M_PI / 180.0);

                double  cv_x = 15 * angle_cc;
                double  cv_y = 15 * angle_cs;



                bool conditionMetOnce = false;

                while (ros::ok())
                {
                    if (!conditionMetOnce) {
                        // 조건을 체크해서 참이면 플래그를 설정
                        if (fabs(current_pose.pose.position.z - position_z) < 0.5) {
                        conditionMetOnce = true;
                        }
                    }

                    // 현재의 z 높이를 체크하여 1.3m 이상인 경우에만 조건을 검사합니다.
                    bool is_above_one_meter = current_pose.pose.position.z >= 1.3;

                    if (conditionMetOnce)
                    {
                        if (lidar_right && is_above_one_meter)
                        {
                            ROS_WARN("Right Lidar alarm triggered! Obstacle detected.");
                            
                            geometry_msgs::Twist velocity_msg;

                            velocity_msg.linear.x = v_x; // avoid
                            velocity_msg.linear.y = v_y;
                            velocity_msg.linear.z = 0.0;
                            
                            velocity_pub.publish(velocity_msg);
                        }
                        else if (lidar_left && is_above_one_meter)
                        {
                            ROS_WARN("Left Lidar alarm triggered! Obstacle detected.");
                            
                            geometry_msgs::Twist velocity_msg;

                            velocity_msg.linear.x = rv_x; // avoid
                            velocity_msg.linear.y = rv_y;
                            velocity_msg.linear.z = 0.0;
                            
                            velocity_pub.publish(velocity_msg);
                        }
                        else if (lidar_center && is_above_one_meter)
                        {

                            ROS_WARN("center Lidar alarm triggered! Obstacle detected.");

                            geometry_msgs::Twist velocity_msg;

                            velocity_msg.linear.x = cv_x;
                            velocity_msg.linear.y= cv_y;
                            velocity_msg.linear.z= 0.0;

                            velocity_pub.publish(velocity_msg);
                        }
                        else
                        {
                            geometry_msgs::PoseStamped pose_msg;

                            pose_msg.header.seq = 0;
                            pose_msg.header.stamp = ros::Time::now();
                            pose_msg.header.frame_id = "";

                            pose_msg.pose.position.x = x_ll;    // 원래 변수  x_l
                            pose_msg.pose.position.y = y_ll;    // 원래 변수 y_l
                            pose_msg.pose.position.z = position_z;

                            pose_msg.pose.orientation.x = quat.x;
                            pose_msg.pose.orientation.y = quat.y;
                            pose_msg.pose.orientation.z = quat.z;
                            pose_msg.pose.orientation.w = quat.w;

                            setpoint_pub.publish(pose_msg);

                            // 이 부분에서 x_l, y_l, position_z에 도달했을 때 "COMPLEX" 메시지 출력 
                            if (fabs(current_pose.pose.position.x - x_ll) < 0.4 &&   // 원래 변수  x_l
                                fabs(current_pose.pose.position.y - y_ll) < 0.4)      // 원래 변수 y_l   
                            {
                                geometry_msgs::PoseStamped pose_msg;

                                
                                pose_msg.pose.position.x = x_ll;  // 원래 변수  x_l
                                pose_msg.pose.position.y = y_ll; // 원래 변수 y_l
                                pose_msg.pose.position.z = land_z;

                                pose_msg.pose.orientation.x = quat.x;
                                pose_msg.pose.orientation.y = quat.y;
                                pose_msg.pose.orientation.z = quat.z;
                                pose_msg.pose.orientation.w = quat.w;                                

                                ROS_INFO("COMPLEX land");
                                setpoint_pub.publish(pose_msg);


                            }
                        }
                    }
                    else
                    {  
                        geometry_msgs::PoseStamped pose_msg;

                        pose_msg.pose.position.x = current_pose.pose.position.x;
                        pose_msg.pose.position.y = current_pose.pose.position.y;
                        pose_msg.pose.position.z = position_z;

                        setpoint_pub.publish(pose_msg);
                    }

                    ros::spinOnce();
                    rate.sleep();
                    
                    //location info
                    ROS_INFO("Current Position: [x: %.2f, y: %.2f, z: %.2f]",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                }



            break;
        }
        case 2:{
            
            // 기존 코드를 여기에 삽입합니다...

                double x1, y1;
                std::cout << "current location (x,y): ";
                std::cin >> x1 >> y1;

                double x2 = 0, y2 = 0; // 원점

                double dx = x2 - x1;
                double dy = y2 - y1;


                double position_x=0, position_y=0, position_z=3;
                double distance = 0;
                double land_z;

                double velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0; // 초기값 설정

                std::cout << "Enter position z(1.6m<z): ";
                std::cin >> position_z;

                std::cout << "LAND position z(1.2m>z): ";
                std::cin >> land_z;




                double tt = 3.14;
                
                double result_radian = atan2(dy, dx);


                Quaternion quat = eulerToQuaternion(result_radian);


                std::cout << "ENU quaternion: ";
                std::cout << "x: " << quat.x << ", y: " << quat.y << ", z: " << quat.z << ", w: " << quat.w << std::endl;
                


                double result_degrees = result_radian * (180.0 / M_PI);

                ////////////////////////////////////////

                double a_total = 120 + result_degrees;

                double angle_s = sin(a_total * M_PI / 180.0);
                double angle_c = cos(a_total * M_PI / 180.0);

                double v_x = 15 * angle_c;
                double v_y = 15 * angle_s;

                /////////////////////////////////////////

                double reversea_total = -120 + result_degrees;

                double angle_rs = sin(reversea_total * M_PI / 180.0);
                double angle_rc = cos(reversea_total * M_PI / 180.0);

                double rv_x = 15 * angle_rc;
                double rv_y = 15 * angle_rs;

                ////////////////////////////////////

                double center_total = 180 + result_degrees;

                double  angle_cs = sin(center_total * M_PI / 180.0);
                double  angle_cc = cos(center_total * M_PI / 180.0);

                double  cv_x = 15 * angle_cc;
                double  cv_y = 15 * angle_cs;



                bool conditionMetOnce = false;

                while (ros::ok())
                {
                    if (!conditionMetOnce) {
                        // 조건을 체크해서 참이면 플래그를 설정
                        if (fabs(current_pose.pose.position.z - position_z) < 0.5) {
                        conditionMetOnce = true;
                        }
                    }

                    // 현재의 z 높이를 체크하여 1.3m 이상인 경우에만 조건을 검사합니다.
                    bool is_above_one_meter = current_pose.pose.position.z >= 1.3;

                    if (conditionMetOnce)
                    {
                        if (lidar_right && is_above_one_meter)
                        {
                            ROS_WARN("Right Lidar alarm triggered! Obstacle detected.");
                            
                            geometry_msgs::Twist velocity_msg;

                            velocity_msg.linear.x = v_x; // avoid
                            velocity_msg.linear.y = v_y;
                            velocity_msg.linear.z = 0.0;
                            
                            velocity_pub.publish(velocity_msg);
                        }
                        else if (lidar_left && is_above_one_meter)
                        {
                            ROS_WARN("Left Lidar alarm triggered! Obstacle detected.");
                            
                            geometry_msgs::Twist velocity_msg;

                            velocity_msg.linear.x = rv_x; // avoid
                            velocity_msg.linear.y = rv_y;
                            velocity_msg.linear.z = 0.0;
                            
                            velocity_pub.publish(velocity_msg);
                        }
                        else if (lidar_center && is_above_one_meter)
                        {

                            ROS_WARN("center Lidar alarm triggered! Obstacle detected.");

                            geometry_msgs::Twist velocity_msg;

                            velocity_msg.linear.x = cv_x;
                            velocity_msg.linear.y= cv_y;
                            velocity_msg.linear.z= 0.0;

                            velocity_pub.publish(velocity_msg);
                        }
                        else
                        {
                            geometry_msgs::PoseStamped pose_msg;

                            pose_msg.header.seq = 0;
                            pose_msg.header.stamp = ros::Time::now();
                            pose_msg.header.frame_id = "";

                            pose_msg.pose.position.x = 0;    // 원래 변수  x_l
                            pose_msg.pose.position.y = 0;    // 원래 변수 y_l
                            pose_msg.pose.position.z = position_z;

                            pose_msg.pose.orientation.x = quat.x;
                            pose_msg.pose.orientation.y = quat.y;
                            pose_msg.pose.orientation.z = quat.z;
                            pose_msg.pose.orientation.w = quat.w;

                            setpoint_pub.publish(pose_msg);

                            // 이 부분에서 x_l, y_l, position_z에 도달했을 때 "COMPLEX" 메시지 출력 
                            if (fabs(current_pose.pose.position.x - 0) < 0.4 &&   // 원래 변수  x_l
                                fabs(current_pose.pose.position.y - 0) < 0.4)      // 원래 변수 y_l   
                            {
                                geometry_msgs::PoseStamped pose_msg;

                                
                                pose_msg.pose.position.x = 0;  // 원래 변수  x_l
                                pose_msg.pose.position.y = 0; // 원래 변수 y_l
                                pose_msg.pose.position.z = land_z;

                                pose_msg.pose.orientation.x = quat.x;
                                pose_msg.pose.orientation.y = quat.y;
                                pose_msg.pose.orientation.z = quat.z;
                                pose_msg.pose.orientation.w = quat.w;                                

                                ROS_INFO("COMPLEX land");
                                setpoint_pub.publish(pose_msg);


                            }
                        }
                    }
                    else
                    {  
                        geometry_msgs::PoseStamped pose_msg;

                        pose_msg.pose.position.x = current_pose.pose.position.x;
                        pose_msg.pose.position.y = current_pose.pose.position.y;
                        pose_msg.pose.position.z = position_z;

                        setpoint_pub.publish(pose_msg);
                    }

                    ros::spinOnce();
                    rate.sleep();
                    
                    //location info
                    ROS_INFO("Current Position: [x: %.2f, y: %.2f, z: %.2f]",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);
                }
            


            break;
        }
        default:
            std::cout << "Invalid input" << std::endl;
            return 1; // 프로그램 종료
    }        



    return 0;
}