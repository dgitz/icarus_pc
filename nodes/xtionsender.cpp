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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    OpenNI2Grabber grabber;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub_ = it.advertise("/camera/rgb/color", 1);;
    image_transport::Publisher depth_pub_ = it.advertise("/camera/depth_registered/image_rect",1);
    cv_bridge::CvImage color_msg;
    cv_bridge::CvImage depth_msg;

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
            // update the display for boout_msgth frames
            if(grabber.getDepthFrame(depthImage))
            {
                // multiply the 11-bit depth values by 32 to extend the color range to a full 16-bits
                depthImage.convertTo(depthImageDraw, -1, 32);
                //imshow("xtion-depth", depthImageDraw);
                depth_msg.image = depthImage;
		depth_msg.header.stamp = ros::Time::now();
		depth_msg.header.seq++;
		depth_msg.encoding = "mono16";
		depth_pub_.publish(depth_msg.toImageMsg());
            }
            if(grabber.getColorFrame(colorImage))
            {
                //imshow("xtion-rgb", colorImage);
                // publish image (go to the CHOPPER!!)
                color_msg.image = colorImage;
		color_msg.header.stamp = ros::Time::now();
		color_msg.header.seq++;
		color_msg.encoding = "bgr8";
                image_pub_.publish(color_msg.toImageMsg());
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

