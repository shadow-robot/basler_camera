#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

#include "image_publisher.h"

using namespace Pylon;

// Adjust value to make it comply with range and increment passed.
//
// The parameter's minimum and maximum are always considered as valid values.
// If the increment is larger than one, the returned value will be: min + (n * inc).
// If the value doesn't meet these criteria, it will be rounded down to ensure compliance.
int64_t Adjust(int64_t val, int64_t minimum, int64_t maximum, int64_t inc)
{
    // Check the input parameters.
    if (inc <= 0)
    {
        // Negative increments are invalid.
        throw LOGICAL_ERROR_EXCEPTION("Unexpected increment %d", inc);
    }
    if (minimum > maximum)
    {
        // Minimum must not be bigger than or equal to the maximum.
        throw LOGICAL_ERROR_EXCEPTION("minimum bigger than maximum.");
    }

    // Check the lower bound.
    if (val < minimum)
    {
        return minimum;
    }

    // Check the upper bound.
    if (val > maximum)
    {
        return maximum;
    }

    // Check the increment.
    if (inc == 1)
    {
        // Special case: all values are valid.
        return val;
    }
    else
    {
        // The value must be min + (n * inc).
        // Due to the integer division, the value will be rounded down.
        return minimum + ( ((val - minimum) / inc) * inc );
    }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "basler_camera");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  camera_info_manager::CameraInfoManager cinfo_manager_(nh);

  int frame_rate;
  if( !priv_nh.getParam("frame_rate", frame_rate) )
    frame_rate = 20;

  int cam_width;
  if( !priv_nh.getParam("width", cam_width) )
    cam_width = 1280;

  int cam_height;
  if( !priv_nh.getParam("height", cam_height) )
    cam_height = 960;

  string camera_info_url;
  if( !priv_nh.getParam("camera_info_url", camera_info_url) )
    camera_info_url = "";

  string frame_id;
  if( !priv_nh.getParam("frame_id", frame_id) )
    frame_id = "";


  int exitCode = 0;

  // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
  // is initialized during the lifetime of this object.
  Pylon::PylonAutoInitTerm autoInitTerm;
  CGrabResultPtr ptrGrabResult;

  try
  {
    // Create an instant camera object for the camera device found first.
    CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());

    ROS_INFO_STREAM("using device " << camera.GetDeviceInfo().GetModelName());

    camera.GrabCameraEvents = true;
    if (camera_info_url != "")
    {
      cinfo_manager_.loadCameraInfo(camera_info_url);
    }

    sensor_msgs::CameraInfo::Ptr cinfo(
      new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

    camera.RegisterImageEventHandler( new ImagePublisher(nh, cinfo, frame_id), RegistrationMode_Append, Cleanup_Delete);

    // Register the standard configuration event handler for enabling software triggering.
    camera.RegisterConfiguration( new CAcquireContinuousConfiguration , RegistrationMode_ReplaceAll, Cleanup_Delete);

    camera.Open();

    GenApi::CIntegerPtr width(camera.GetNodeMap().GetNode("Width"));
    GenApi::CIntegerPtr height(camera.GetNodeMap().GetNode("Height"));

    int64_t newWidth = cinfo->width;
    newWidth = Adjust(newWidth, width->GetMin(), width->GetMax(), width->GetInc());

    int64_t newHeight = cinfo->height;
    newHeight = Adjust(newHeight, height->GetMin(), height->GetMax(), height->GetInc());

    width->SetValue(newWidth);
    height->SetValue(newHeight);

    /*ROS_INFO_STREAM("GetValue() " << width->GetValue());
    ROS_INFO_STREAM("GetMin() " << width->GetMin());
    ROS_INFO_STREAM("GetMax() " << width->GetMax());
    ROS_INFO_STREAM("width->GetInc() " << width->GetInc());

    ROS_INFO_STREAM("GetValue() " << height->GetValue());
    ROS_INFO_STREAM("GetMin() " << height->GetMin());
    ROS_INFO_STREAM("GetMax() " << height->GetMax());
    ROS_INFO_STREAM("width->GetInc() " << height->GetInc());*/

    // Start the grabbing
    camera.StartGrabbing();

    while ( camera.IsGrabbing() && ros::ok())
    {
      camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
      ros::spinOnce();
    }
  }
  catch (GenICam::GenericException &e)
  {
    ROS_ERROR_STREAM ( "An exception occurred." << e.GetDescription());
    exitCode = 1;
  }
  return exitCode;
}

