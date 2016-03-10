#ifndef INCLUDED_IMAGEPUBLISHER
#define INCLUDED_IMAGEPUBLISHER

#include <pylon/ImageEventHandler.h>
#include <pylon/GrabResultPtr.h>

#include <pylon/PylonIncludes.h>
#include <pylon/PylonImage.h>
#include <pylon/Pixel.h>
#include <pylon/ImageFormatConverter.h>

#include <ros/console.h>
#include <camera_info_manager/camera_info_manager.h>

namespace Pylon
{
  using namespace GenApi;
  using namespace std;

  class ImagePublisher : public CImageEventHandler
  {
  public:

    ImagePublisher(ros::NodeHandle nh, sensor_msgs::CameraInfo::Ptr cinfo, string frame_id)
      : nh_(nh), it_(nh_) {
      cam_pub_ = it_.advertiseCamera("image_raw", 1);
      converter_.OutputPixelFormat = PixelType_RGB8packed;
      cinfo_ = cinfo;
      frame_id_ = frame_id;
    }

    virtual void OnImagesSkipped( CInstantCamera& camera, size_t countOfSkippedImages)
    {
      ROS_ERROR_STREAM (countOfSkippedImages  << " images have been skipped.");
    }

    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
      ROS_DEBUG_STREAM("*OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName());

      ros::Time timestamp = ros::Time::now();
      GenApi::CIntegerPtr width(camera.GetNodeMap().GetNode("Width"));
      GenApi::CIntegerPtr height(camera.GetNodeMap().GetNode("Height"));
      cv::Mat cv_img(width->GetValue(), height->GetValue(), CV_8UC3);

      if (ptrGrabResult->GrabSucceeded())
      {
        converter_.Convert(pylon_image_, ptrGrabResult);

        cv_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3,(uint8_t*)pylon_image_.GetBuffer());
        sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_img).toImageMsg();

        sensor_msgs::CameraInfo::Ptr cinfo = cinfo_;

        image->header.frame_id = frame_id_;
        image->header.stamp = timestamp;
        cinfo->header.frame_id = frame_id_;
        cinfo->header.stamp = timestamp;

        cam_pub_.publish(image, cinfo);
      }
      else
      {
        ROS_ERROR_STREAM("Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription());
      }
    }
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraPublisher cam_pub_;
    sensor_msgs::CameraInfo::Ptr cinfo_;
    string frame_id_;

    CImageFormatConverter converter_;
    CPylonImage pylon_image_;
    EPixelType pixel_type_;
  };
}

#endif /* INCLUDED_IMAGEPUBLISHER */
