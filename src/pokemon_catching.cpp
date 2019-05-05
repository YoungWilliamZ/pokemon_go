#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;
int fileNum = 1;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  // image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image", 1,
      &ImageConverter::imageCb, this);
    // image_pub_ = it_.advertise("cmd_vel", 1);
  }

  void saveImg(std_msgs::Bool save){
	  if(save.data){
      stringstream stream;
          stream <<"./pokemon" << fileNum <<".jpg";
      imwrite(stream.str(),img);
      cout <<"pokemon" << fileNum << " had Saved."<< endl;
      fileNum++;
	  }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;
    double depth = cv_ptr->image.at<double>(cv::Point(height/2,width/2));//you can change 240,320 to your interested pixel
    ROS_INFO(" the middle distance: [%f]", depth);

    


    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pokemon_catching");
    ros::NodeHandle n;
    ImageConverter ic;
    ros::spin();
    return 0;
}