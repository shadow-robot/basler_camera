#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

#include "image_publisher.h"

using namespace Pylon;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "basler_camera");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  camera_info_manager::CameraInfoManager cinfo_manager_(nh);

  int frame_rate;
  if( !priv_nh.getParam("frame_rate", frame_rate) )
    frame_rate = 20;

  string camera_info_url;
  if( !priv_nh.getParam("camera_info_url", camera_info_url) )
    camera_info_url = "";

  string frame_id;
  if( !priv_nh.getParam("frame_id", frame_id) )
    frame_id = "";

  std::string serial_number;
  if( !priv_nh.getParam("serial_number", serial_number) )
    serial_number = "";

  int exitCode = 0;

  // Automatically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
  // is initialized during the lifetime of this object.
  Pylon::PylonAutoInitTerm autoInitTerm;
  CGrabResultPtr ptrGrabResult;

  try
  {
    CTlFactory& tlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    if ( tlFactory.EnumerateDevices(devices) == 0 )
    {
      throw RUNTIME_EXCEPTION( "No camera present.");
    }

    CInstantCamera camera;
    if (serial_number == "") {
      // Create an instant camera object for the camera device found first.
      camera.Attach(CTlFactory::GetInstance().CreateFirstDevice());
    } else {
      // Look up the camera by its serial number
      for (size_t i=0; i<devices.size(); i++) {
        if (devices[i].GetSerialNumber().c_str() == serial_number) {
          camera.Attach( tlFactory.CreateDevice(devices[i]));
          break;
        }
      }
    }

    ROS_INFO_STREAM("using device " << camera.GetDeviceInfo().GetModelName());

    camera.GrabCameraEvents = true;
    if (camera_info_url != "")
    {
      cinfo_manager_.loadCameraInfo(camera_info_url);
    }

    sensor_msgs::CameraInfo::Ptr cinfo(
      new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

    camera.RegisterImageEventHandler( new ImagePublisher(nh, cinfo, frame_id), RegistrationMode_Append, Cleanup_Delete);

    camera.RegisterConfiguration( new CAcquireContinuousConfiguration , RegistrationMode_ReplaceAll, Cleanup_Delete);

    camera.Open();

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

