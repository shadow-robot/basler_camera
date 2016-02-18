#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
//#include <pylon/usb/BaslerUsbInstantCamera.h>

#include "image_publisher.h"

using namespace Pylon;
using namespace GenApi;

void handle_basler_int_parameters(CInstantCamera& camera, std::map<std::string, int> int_map)
{
  INodeMap& nodemap = camera.GetNodeMap();
  try
  {
    for (std::map<std::string,int>::iterator it=int_map.begin(); it!=int_map.end(); ++it)
    {
      CIntegerPtr this_node(nodemap.GetNode(it->first.c_str()));
      if (!IsWritable(this_node))
      {
        ROS_ERROR_STREAM("Basler parameter '" << it->first << "' isn't writable.");
        continue;
      }
      this_node->SetValue(it->second);
    }
  }
  catch (const GenericException& e)
  {
    ROS_ERROR_STREAM(e.GetDescription());
  }
}

void handle_basler_float_parameters(CInstantCamera& camera, std::map<std::string, float> float_map)
{
  INodeMap& nodemap = camera.GetNodeMap();
  try
  {
    for (std::map<std::string,float>::iterator it=float_map.begin(); it!=float_map.end(); ++it)
    {
      CFloatPtr this_node(nodemap.GetNode(it->first.c_str()));
      if (!IsWritable(this_node))
      {
        ROS_ERROR_STREAM("Basler parameter '" << it->first << "' isn't writable.");
        continue;
      }
      this_node->SetValue(it->second);
    }
  }
  catch (const GenericException& e)
  {
    ROS_ERROR_STREAM(e.GetDescription());
  }
}

void handle_basler_enum_parameters(CInstantCamera& camera, std::map<std::string, std::string> enum_map)
{
  INodeMap& nodemap = camera.GetNodeMap();
  try
  {
    for (std::map<std::string,std::string>::iterator it=enum_map.begin(); it!=enum_map.end(); ++it)
    {
      CEnumerationPtr this_node(nodemap.GetNode(it->first.c_str()));
      if (!IsWritable(this_node))
      {
        ROS_ERROR_STREAM("Basler parameter '" << it->first << "' isn't writable.");
        continue;
      }
      if (!IsAvailable(this_node->GetEntryByName(it->second.c_str())))
      {
        ROS_ERROR_STREAM("Valuer '" << it->second << "' isn't available for basler param '" << it->first << "'.");
        continue;
      }
      this_node->FromString(it->second.c_str());
    }
  }
  catch (const GenericException& e)
  {
    ROS_ERROR_STREAM(e.GetDescription());
  }
}

void handle_basler_parameters(CInstantCamera& camera)
{
  ros::NodeHandle private_handle("~");
  std::map<std::string, std::string> enum_params;
  std::map<std::string, float> float_params;
  std::map<std::string, int> int_params;

  // if (private_handle.hasParam("basler_params_enum"))
  // {
  //   private_handle.getParam("basler_params_enum", enum_params);
  //   handle_basler_enum_parameters(camera, enum_params);
  // }

  if (private_handle.hasParam("basler_params_int"))
  {
    private_handle.getParam("basler_params_int", int_params);
    handle_basler_int_parameters(camera, int_params);
  }

  if (private_handle.hasParam("basler_params_float"))
  {
    private_handle.getParam("basler_params_float", float_params);
    handle_basler_float_parameters(camera, float_params);
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

    INodeMap& nodemap = camera.GetNodeMap();
    CIntegerPtr frame_rate_node(nodemap.GetNode("AcquisitionFrameRate"));
    frame_rate_node->SetValue(frame_rate); // This would be overwriten if you specifed different
    // frame rate as basler_float_param but left here to honour previously documented although
    // apparently unimplemented feature...


    handle_basler_parameters(camera);

/*

    CFloatPtr max_exposure_node(nodemap.GetNode("AutoExposureUpperLimit"));
*/



      // if linked mode

      // set max exposure time to work with frame rate

    camera.StartGrabbing();

    while ( camera.IsGrabbing() && ros::ok())
    {
      camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
      ros::spinOnce();
    }
  }
  catch (GenICam::GenericException &e)
  {   ROS_ERROR_STREAM ( "An exception occurred." << e.GetDescription());
    exitCode = 1;
  }
  return exitCode;
}
