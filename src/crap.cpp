#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <opencv2/core/core.hpp>

#include "OpenNI2Grabber.h"

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("out", 1);

    cv:namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point
***********************************************************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");
  ImageConverter ic;

   OpenNI2Grabber grabber;

    bool isRunning = false;
    const int timeoutMs = 1000;

    // attempt to start the grabber
    if(grabber.initialize())
    {
        if(grabber.start())
        {
            isRunning = grabber.isRunning();
        }
    }
    else
    {
        std::cout << "Unable to initialize OpenNI2Grabber, program terminating!" << std::endl;
        return 0;
    }

    // acquire frames until program termination
    std::cout << "Press 'q' to halt acquisition..." << std::endl;
    Mat depthImage, colorImage;
    Mat depthImageDraw;
    while(isRunning)
    {
        // wait for a new image frame
        if(grabber.waitForFrame(timeoutMs))
        {
            // update the display for both frames
            if(grabber.getDepthFrame(depthImage))
            {
                // multiply the 11-bit depth values by 32 to extend the color range to a full 16-bits
                depthImage.convertTo(depthImageDraw, -1, 32);
                imshow("depth", depthImageDraw);
            }
            if(grabber.getColorFrame(colorImage))
            {
                imshow("rgb", colorImage);
                //Convert Image Here!
                
            }
        }
        else
        {
            std::cout << "No new frames received in " << timeoutMs << " ms..." << std::endl;
        }

        // check for program termination
        char key = (char) cv::waitKey(1);
        if(key == 'q' || key == 'Q')
        {
            std::cout << "Terminating program..." << std::endl;
            isRunning = false;
        }
    }

    // stop the acquisition
    grabber.stop();

    return 0;
}

