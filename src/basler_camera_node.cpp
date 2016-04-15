#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

#include "image_publisher.h"
#include <XmlRpcValue.h>

using namespace Pylon;
using namespace GenApi;
using std::string;

void handle_basler_int_parameter(CInstantCamera& camera, string name, int value)
{
  INodeMap& nodemap = camera.GetNodeMap();
  try
  {
    ROS_INFO_STREAM("Setting int param " << name << " to " << value << ".");
    CIntegerPtr this_node(nodemap.GetNode(name.c_str()));
    if (!IsWritable(this_node))
    {
      ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
      return;
    }
    this_node->SetValue(value);
  }
  catch (const GenericException& e)
  {
    ROS_ERROR_STREAM(e.GetDescription());
  }
}

void handle_basler_float_parameter(CInstantCamera& camera, string name, double value)
{
  INodeMap& nodemap = camera.GetNodeMap();
  try
  {
    ROS_INFO_STREAM("Setting float param " << name << " to " << value << ".");
    CFloatPtr this_node(nodemap.GetNode(name.c_str()));
    if (!IsWritable(this_node))
    {
      ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
      return;
    }
    this_node->SetValue(value);
  }
  catch (const GenericException& e)
  {
    ROS_ERROR_STREAM(e.GetDescription());
  }
}

void handle_basler_enum_parameter(CInstantCamera& camera, string name, string value)
{
  INodeMap& nodemap = camera.GetNodeMap();
  try
  {
    ROS_INFO_STREAM("Setting enum param " << name << " to " << value << ".");
    CEnumerationPtr this_node(nodemap.GetNode(name.c_str()));
    if (!IsWritable(this_node))
    {
      ROS_ERROR_STREAM("Basler parameter '" << name << "' isn't writable or doesn't exist.");
      return;
    }
    if (!IsAvailable(this_node->GetEntryByName(value.c_str())))
    {
      ROS_ERROR_STREAM("Valuer '" << value << "' isn't available for basler param '" << name << "'.");
      return;
    }
    this_node->FromString(value.c_str());
  }
  catch (const GenericException& e)
  {
    ROS_ERROR_STREAM(e.GetDescription());
  }
}

void handle_basler_parameter(CInstantCamera& camera, XmlRpc::XmlRpcValue& param)
{
  string type = param["type"];
  if ("int" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeInt,
                   "Type of value for %s must be int", string(param["name"]).c_str());
    handle_basler_int_parameter(camera, param["name"], param["value"]);
  }
  else if ("float" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeDouble,
                   "Type of value for %s must be float", string(param["name"]).c_str());
    handle_basler_float_parameter(camera, param["name"], param["value"]);
  }
  else if ("enum" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeString,
                   "Type of value for %s must be string", string(param["name"]).c_str());
    handle_basler_enum_parameter(camera, param["name"], param["value"]);
  }
  else
  {
    ROS_FATAL_STREAM("Unknown param type for parameter " << param["name"] << ": " << type);
  }
}

void handle_basler_parameters(CInstantCamera& camera)
{
  ros::NodeHandle private_handle("~");
  XmlRpc::XmlRpcValue params;
  private_handle.getParam("basler_params", params);
  ROS_ASSERT_MSG(params.getType() == XmlRpc::XmlRpcValue::TypeArray, "Badly formed basler param yaml");
  for (size_t index = 0; index < params.size(); ++index)
  {
    ROS_ASSERT_MSG(params[index].getType() == XmlRpc::XmlRpcValue::TypeStruct, "Badly formed basler param yaml");
    ROS_ASSERT_MSG(params[index].hasMember("name"), "Param needs name");
    ROS_ASSERT_MSG(params[index].hasMember("type"), "Param needs type");
    ROS_ASSERT_MSG(params[index].hasMember("value"), "Param needs value");

    handle_basler_parameter(camera, params[index]);
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

    handle_basler_float_parameter(camera, "AcquisitionFrameRate", frame_rate); // This would be overwriten if you specifed
    // different frame rate via a yaml file, here to honour previously documented although apparently unimplemented feature.

    handle_basler_parameters(camera);

    // Set the pixel format to RGB8 if available.
    INodeMap& nodemap = camera.GetNodeMap();
    CEnumerationPtr pixelFormat( nodemap.GetNode( "PixelFormat"));
    String_t oldPixelFormat = pixelFormat->ToString();
    if ( IsAvailable( pixelFormat->GetEntryByName( "RGB8")))
    {
        pixelFormat->FromString( "RGB8");
    }

    camera.StartGrabbing();

    while ( camera.IsGrabbing() && ros::ok())
    {
      camera.RetrieveResult( 1, ptrGrabResult, TimeoutHandling_Return);
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
