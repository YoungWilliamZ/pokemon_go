#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <iostream>
#include <std_msgs/Bool.h>

// using namespace cv;
// using namespace std;

static const std::string OPENCV_WINDOW = "Pokemon Catch";
namespace enc = sensor_msgs::image_encodings;
int fileNum = 1;
int count = 0;
float lastSpeed = 0;
bool isStart = false;
float depth = -1;


class DepthCatching
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher save_pub_;

public:
  ros::Publisher pub; 
  DepthCatching()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    ROS_INFO("Start Contruct");
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,      
	    &DepthCatching::imageCb, this);
    // image_pub_ = it_.advertise("cmd_vel", 1);
    pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
    save_pub_ = nh_.advertise<std_msgs::Bool>("/pokemon_go/save", 1);
    ROS_INFO("Complete Contruct");

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~DepthCatching()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
    ROS_INFO("DeContruct");
  }

  void shutdown()
  {
    pub.publish(geometry_msgs::Twist());//使机器人停止运动
    // ROS_INFO("move_turtle_goforward ended!");
    std_msgs::Bool saveImage;
        saveImage.data = true;
        save_pub_.publish(saveImage);
    ROS_INFO("Catch the pokemon!");
    ros::shutdown();
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("Start detect image depth: time = [%d]", count);
    count++;
  
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;
    
    depth = cv_ptr->image.at<float>(height/2,width/2);//you can change 240,320 to your interested pixel
    for (int i = height/3; i < height*2/3; i++)
    {
      for (int j = width/3; j < width*2/3; j++)
      {
        depth = depth < cv_ptr->image.at<float>(i,j) ? cv_ptr->image.at<float>(i,j) : depth;
      }
      
    }
    
    // std::vector<char> my_vec(cv_ptr->image.begin(), cv_ptr->image.end());
    // ROS_INFO(" the height: [%d]", height);
    // ROS_INFO(" the width : [%d]", width);
    ROS_INFO(" the middle distance: [%f]", depth);
    // ROS_INFO(" the data: [%s]", cv_ptr->image.data);
    
    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);
  }
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pokemon_catching");
    DepthCatching dc;
    // ros::spin(); // loop to call callback
    ros::Rate loopRate(10);//ros::Rate对象可以允许你指定自循环的频率

    while (ros::ok()){
      ros::spinOnce();
      if (depth == -1)
      {
        continue;
      }
      
      geometry_msgs::Twist speed; // 控制信号载体 Twist message
      speed.linear.x = 0;
      if (!isStart) {
        if (depth > 575) {
            speed.linear.x = 0.1;
          // 设置线速度为0.1m/s，正为前进，负为后退
        } else if (depth < 525) {
            speed.linear.x = -0.1;
        } else {
            dc.shutdown();
        }
        isStart = true;
        lastSpeed = speed.linear.x;
      } else {
        speed.linear.x = lastSpeed;
        if (depth <= 575 && depth >= 525){
          dc.shutdown();
          break;
        }
        
      }
      speed.angular.z = 0; // 设置角速度为0rad/s，正为左转，负为右转
      dc.pub.publish(speed); // 将刚才设置的指令发送给机器人
      loopRate.sleep();//休眠直到一个频率周期的时间
      
      // for (int i = 0; i < 9999999; i++)
      // {
      //   /* code */
      // }
      
      
    }
  
    return 0;
}
