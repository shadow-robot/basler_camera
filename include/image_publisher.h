#ifndef INCLUDED_IMAGEPUBLISHER
#define INCLUDED_IMAGEPUBLISHER

#include <pylon/ImageEventHandler.h>
#include <pylon/GrabResultPtr.h>

#include <pylon/PylonIncludes.h>
#include <pylon/PylonImage.h>
#include <pylon/Pixel.h>
#include <pylon/ImageFormatConverter.h>

#include <ros/console.h>

namespace Pylon
{
  //class CInstantCamera;

  class ImagePublisher : public CImageEventHandler
  {
  public:

    ImagePublisher(ros::NodeHandle nh)
      : nh_(nh), it_(nh_) {
      //cam_pub_ = it_.advertiseCamera("image_raw", 1, false);
      cam_pub_ = it_.advertise("camera/image", 1);
      converter_.OutputPixelFormat = PixelType_RGB8packed;
    }

    virtual void OnImagesSkipped( CInstantCamera& camera, size_t countOfSkippedImages)
    {
      ROS_WARN_STREAM (countOfSkippedImages  << " images have been skipped.");
    }


    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
      ROS_DEBUG_STREAM("OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName());

      GenApi::CIntegerPtr width(camera.GetNodeMap().GetNode("Width"));
      GenApi::CIntegerPtr height(camera.GetNodeMap().GetNode("Height"));
      cv::Mat cv_img(width->GetValue(), height->GetValue(), CV_8UC3);

      // Image grabbed successfully?
      if (ptrGrabResult->GrabSucceeded())
      {
        converter_.Convert(pylon_image_, ptrGrabResult);

        cv_img = cv::Mat(ptrGrabResult->GetHeight(),     ptrGrabResult->GetWidth(), CV_8UC3,(uint8_t*)pylon_image_.GetBuffer());
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_img).toImageMsg();
        cam_pub_.publish(msg);
      }
      else
      {
        ROS_ERROR_STREAM("Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription());
      }
    }
  private:
    ros::NodeHandle nh_;
    ros::Publisher camera_pub;
    image_transport::ImageTransport it_;
    image_transport::Publisher cam_pub_;

    CImageFormatConverter converter_;
    CPylonImage pylon_image_;
    EPixelType pixel_type_;
  };
}

#endif /* INCLUDED_IMAGEPUBLISHER */
