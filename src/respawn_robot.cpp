// #include <respawn_robot.h>

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <respawn_robot/dataset.h>

#include <random>
#include <iostream>
#include <cmath>

int respawn_flag = 1;
bool get_s_or_f = 0;
double x = 0;
double y = 0;
double z = 0;
double x_thres = 0.05;
double y_thres = 0.05;
double roll_limit = 0.7;
double pitch_limit = 0.7;
double roll = 0;
double pitch = 0;
double yaw = 0;
int data_id = 0;
bool s_or_f;
int cnt = 0;
double timer1 = 0;
double timer2 = 0;
double timer3 = 0;
double yaw_target = 0;
double yaw_target_dis = 0;
int k = 0;
double rand_x_tar = 0;
double rand_y_tar = 0;
double x_init = 0;
double y_init = 0;

respawn_robot::dataset dataset;
grid_map_msgs::GridMap elevation_map_raw;
std_msgs::Int8 controlinput;


// Callback function for the robot's odometry or state topic
void msgCallbackBodyPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    x = msg->data[0];
    y = msg->data[1];
    z = msg->data[2];
    roll = msg->data[3];
    pitch = msg->data[4];
    yaw = msg->data[5];

    // if (abs(roll) > roll_limit || abs(pitch) > pitch_limit || 시간이 너무많이 걸린다) {
    //     s_or_f = 0; 
    //     get_s_or_f = 1;
    // }
    // else if ( abs(x - rand_x_tar) < x_thres && abs(y - rand_y_tar) < y_thres) {
    //     s_or_f = 1;
    //     get_s_or_f = 1;
    // }
    // else {
    //     get_s_or_f = 0;
    // }

    if (msg->data[0] < -1) // 로봇이 성공인지 실패인지 여부
    {
        get_s_or_f = 1;
    }
    else {
        get_s_or_f = 0;
    }
}

void msgCallbackElevationMap(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    elevation_map_raw.data = msg->data;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "respawn_robot");

    // Subscribe to the robot's odometry or state topic
    ros::NodeHandle nh;

    ros::Subscriber sub_bodypose = nh.subscribe("/aidin81/BodyPose_sim", 10, msgCallbackBodyPose);
    ros::Subscriber sub_elevationmap = nh.subscribe("/aidin81/elevation_mapping/elevation_map_raw", 10, msgCallbackElevationMap);

    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/aidin81/Path", 100);
    ros::Publisher pub_zerotorqueflag = nh.advertise<std_msgs::Bool>("/aidin81/ZeroTorqueFlag", 100);
    ros::Publisher pub_controlinput = nh.advertise<std_msgs::Int8>("/aidin81/ControlInput", 100);
    ros::Publisher pub_dataset = nh.advertise<respawn_robot::dataset>("/aidin81/dataset", 100);

    ros::Rate rate(2); // 1초에 2번

    while(ros::ok()) {

        if (respawn_flag == 1) { // 1. mpc 켜는 명령 보내기, id 데이터셋에 저장하기.
            timer1 += 0.5;
                
            if(timer1 == 2) {
                std::cout << "initialize imu" << std::endl;
                controlinput.data = 2;
                pub_controlinput.publish(controlinput);
            }
            else if(timer1 == 3) {
                std::cout << "command stand mode"  << std::endl;
                controlinput.data = 4;
                pub_controlinput.publish(controlinput);
            }
            else if(timer1 == 4) {
                std::cout << "cammand mpc mode" << std::endl;
                controlinput.data = 5;
                pub_controlinput.publish(controlinput);

                timer1 = 0;

                dataset.id = data_id;

                respawn_flag = 2;    
            }
        }
        else if (respawn_flag == 2) { // 2. elevation map 데이터셋에 저장하기.
            std::cout << "get elevation map" << std::endl;        
            dataset.elevation_map_raw = elevation_map_raw;
            respawn_flag = 3;
        }
        else if (respawn_flag == 3) {  // 3. 타겟 포지션 보내고 데이터셋에 저장하기.    
            timer2 += 0.5;
            if(timer2 == 0.5) {       
                std::cout << "publish target position" << std::endl;        

                double radius = 5.0;
                double centerX = 0.0;
                double centerY = 0.0;

                // Seed the random number generator
                std::random_device rd_tar;
                std::mt19937 gen_tar(rd_tar());
                std::uniform_real_distribution<> dis_tar(0.0, 1.0);

                // Generate a random radius and angle
                double r = radius * std::sqrt(dis_tar(gen_tar));
                double theta = 2.0 * M_PI * dis_tar(gen_tar);
                
                x_init = x;
                y_init = y;

                // Convert polar coordinates to Cartesian coordinates
                rand_x_tar = x + r * std::cos(theta);
                rand_y_tar = y + r * std::sin(theta);
                
                // target x,y dataset 저장.
                dataset.x = rand_x_tar;
                dataset.y = rand_y_tar;

                yaw_target = atan2(rand_y_tar - y, rand_x_tar - x); // theta based world frame  
                
                yaw_target_dis = yaw_target / 10;


            }
            k++;

            if(k < 11) {
                yaw_target = yaw_target_dis*k;
            }
            else {
                yaw_target = yaw_target_dis*10;
            }
        

            tf2::Quaternion q;

            q.setRPY(0, 0, yaw_target);

            nav_msgs::Path path;

            path.header.stamp = ros::Time::now();
            path.header.frame_id = "world";
            path.poses.resize(2);  // allocate memory for the poses array
            path.poses[0].header.frame_id = "world";
            path.poses[0].header.stamp = ros::Time::now();
            path.poses[0].pose.position.x = x_init;
            path.poses[0].pose.position.y = y_init;
            path.poses[0].pose.orientation.x = 0;
            path.poses[0].pose.orientation.y = 0;
            path.poses[0].pose.orientation.z = 0;
            path.poses[0].pose.orientation.w = 1;

            path.poses[1].header.frame_id = "world";
            path.poses[1].header.stamp = ros::Time::now();
            path.poses[1].pose.position.x = rand_x_tar;
            path.poses[1].pose.position.y = rand_y_tar;
            path.poses[1].pose.orientation.x = q.x();
            path.poses[1].pose.orientation.y = q.y();
            path.poses[1].pose.orientation.z = q.z();
            path.poses[1].pose.orientation.w = q.w();

            pub_path.publish(path);

            if (abs(yaw_target - yaw) < 0.1){  
                k = 0;
                timer2 = 0;

                respawn_flag = 4;
            }
        }
        else if(respawn_flag == 4) { // 4. 성공인지 실패인지 결과가 나오면 데이터셋에 저장하고 퍼블리시하기.
            std::cout << "move ..." << std::endl;        

            if (get_s_or_f == 1) {
                if (s_or_f == 1) {
                    std::cout << "success !" << std::endl; 
                }
                else {
                    std::cout << "fail !" << std::endl; 
                }
                dataset.s_or_f = s_or_f;

                pub_dataset.publish(dataset);
                
                respawn_flag = 5;
            }
        }
        else if(respawn_flag == 5) { // 5. 로봇 crawl 자세로 바꾸기

            controlinput.data = 3;
            pub_controlinput.publish(controlinput);

            // nav_msgs::Path path;

            // path.header.stamp = ros::Time::now();
            // path.header.frame_id = "world";
            // path.poses.resize(2);  // allocate memory for the poses array
            // path.poses[0].header.frame_id = "world";
            // path.poses[0].header.stamp = ros::Time::now();
            // path.poses[0].pose.position.x = x_init;
            // path.poses[0].pose.position.y = y_init;
            // path.poses[0].pose.orientation.x = 0;
            // path.poses[0].pose.orientation.y = 0;
            // path.poses[0].pose.orientation.z = 0;
            // path.poses[0].pose.orientation.w = 1;

            // path.poses[1].header.frame_id = "world";
            // path.poses[1].header.stamp = ros::Time::now();
            // path.poses[1].pose.position.x = x;
            // path.poses[1].pose.position.y = y;
            // path.poses[1].pose.orientation.x = 0;
            // path.poses[1].pose.orientation.y = 0;
            // path.poses[1].pose.orientation.z = 0;
            // path.poses[1].pose.orientation.w = 1;

            // pub_path.publish(path);

            timer3 += 0.5;
            if(timer3 == 0.5) {
                std::cout << "command crawl mode"  << std::endl;        
            }
            if(timer3 == 2) {
                respawn_flag = 6;
                timer3 = 0;            
            }
        }    

        // else if(respawn_flag == 5) { // 5. 로봇 제로토크 보내기
        //     std::cout << "respawn_flag: " << respawn_flag << std::endl;        

        //     // std_msgs::Bool zerotorqueflag;
        //     // zerotorqueflag.data = 1;
        //     // pub_zerotorqueflag.publish(zerotorqueflag);

        //     timer += 0.5;
        
        //     if(timer == 3) {
        //         respawn_flag = 6;
        //         timer = 0;            
        //     }
        // }
        else if(respawn_flag == 6) { // 6. 랜덤 포지션으로 리스폰하기.
            std::cout << "respawn ..." << std::endl;        

            // create a random number generator engine
            std::random_device rd_init;
            std::mt19937 gen_init(rd_init());

            // create a distribution that generates random double numbers in [0.0, 1.0)
            std::uniform_real_distribution<double> dis_init(-5.0, 5.0);

            double rand_x_init = dis_init(gen_init);
            double rand_y_init = dis_init(gen_init);

            gazebo_msgs::ModelState modelState;
            modelState.model_name = "aidin81";  // Replace with your robot's name
            modelState.pose.position.x = rand_x_init;   // Replace with your robot's starting position
            modelState.pose.position.y = rand_y_init;
            modelState.pose.position.z = 1.0;
            modelState.pose.orientation.x = 0.0;  // Replace with your robot's starting orientation
            modelState.pose.orientation.y = 0.0;
            modelState.pose.orientation.z = 0.0;
            modelState.pose.orientation.w = 1.0;
            ros::NodeHandle nh;
            ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
            gazebo_msgs::SetModelState srv;
            srv.request.model_state = modelState;
            if (client.call(srv))
            {
                // ROS_INFO("Robot respawned successfully");
                std::cout << "respawn x: " << rand_x_init << " y: " << rand_y_init << std::endl;        
            }
            else
            {
                ROS_ERROR("Failed to call service /gazebo/set_model_state");
            }

            data_id++;
      
            respawn_flag = 1;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
