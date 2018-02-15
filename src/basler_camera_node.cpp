#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <basler_camera/CameraConfig.h>
#include <dynamic_reconfigure/server.h>

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

#include "image_publisher.h"
#include <XmlRpcValue.h>

using namespace Pylon;
using namespace GenApi;
using std::string;

static CInstantCamera camera;

void handle_basler_boolean_parameter(CInstantCamera& camera, string name, bool value)
{
  INodeMap& nodemap = camera.GetNodeMap();
  try
  {
    ROS_INFO_STREAM("Setting boolean param " << name << " to " << value << ".");
    CBooleanPtr this_node(nodemap.GetNode(name.c_str()));
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
  ros::NodeHandle nh("~");
  string type = param["type"];
  if ("boolean" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeBoolean,
                   "Type of value for %s must be boolean", string(param["name"]).c_str());
    handle_basler_boolean_parameter(camera, param["name"], param["value"]);
    nh.setParam(param["name"], (bool)param["value"]);
  }
  else if ("int" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeInt,
                   "Type of value for %s must be int", string(param["name"]).c_str());
    handle_basler_int_parameter(camera, param["name"], param["value"]);
    nh.setParam(param["name"], (int)param["value"]);
  }
  else if ("float" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeDouble,
                   "Type of value for %s must be float", string(param["name"]).c_str());
    handle_basler_float_parameter(camera, param["name"], param["value"]);
    nh.setParam(param["name"], (double)param["value"]);
  }
  else if ("enum" == type)
  {
    ROS_ASSERT_MSG(param["value"].getType() == XmlRpc::XmlRpcValue::TypeString,
                   "Type of value for %s must be string", string(param["name"]).c_str());
    handle_basler_enum_parameter(camera, param["name"], param["value"]);
    nh.setParam(param["name"], (std::string)param["value"]);
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

void handle_parameters(CInstantCamera& camera)
{
  ros::NodeHandle private_handle("~");
  std::vector<std::string> keys;
  std::string prefix = private_handle.getNamespace() + "/";
  if(!private_handle.getParamNames(keys))
  {
      ROS_INFO("No params found on parameter server, using defaults from dynamic_reconfigure.");
      return;
  }
  for(std::vector<std::string>::iterator k=keys.begin(); k != keys.end(); ++k)
  {
      if((*k).size() < prefix.size())
      {
          continue;
      }
      std::pair<std::string::iterator, std::string::iterator> different = std::mismatch(prefix.begin(), prefix.end(), (*k).begin());
      if(different.first != prefix.end())
      {
          continue;
      }
      std::string param_name(different.second, (*k).end());
      XmlRpc::XmlRpcValue v;
      private_handle.getParam(*k, v);
      if(param_name == "basler_params")
      {
          ROS_WARN("Using legacy basler_params value.");
          handle_basler_parameters(camera);
          break;
      }
      else if((param_name == "camera_info_url") ||
              (param_name == "frame_id") ||
              (param_name == "frame_rate") ||
              (param_name == "serial_number"))
      {
          // handled in main before camera is opened
          continue;
      }
      else if(v.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      {
          handle_basler_boolean_parameter(camera, param_name, v);
      }
      else if(v.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
          handle_basler_int_parameter(camera, param_name, v);
      }
      else if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {
          handle_basler_float_parameter(camera, param_name, v);
      }
      else if(v.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
          handle_basler_enum_parameter(camera, param_name, v);
      }
      else
      {
          ROS_ERROR_STREAM("Unexpected data type for " << param_name);
      }
  }
}

void configure_callback(basler_camera::CameraConfig &config, uint32_t level)
{
    for (std::vector<basler_camera::CameraConfig::AbstractParamDescriptionConstPtr>::const_iterator _i = config.__getParamDescriptions__().begin(); _i != config.__getParamDescriptions__().end(); ++_i)
    {
        boost::any val;
        (*_i)->getValue(config, val);
        if("bool" == (*_i)->type)
        {
            handle_basler_boolean_parameter(camera, (*_i)->name,  boost::any_cast<bool>(val));
        }
        else if("double" == (*_i)->type)
        {
            handle_basler_float_parameter(camera, (*_i)->name,  boost::any_cast<double>(val));
        }
        else if("int" == (*_i)->type)
        {
            handle_basler_int_parameter(camera, (*_i)->name,  boost::any_cast<int>(val));
        }
        else if("str" == (*_i)->type)
        {
            handle_basler_enum_parameter(camera, (*_i)->name,  boost::any_cast<std::string>(val));
        }
        else
        {
            ROS_FATAL_STREAM("Unknown param type for config parameter " << (*_i)->name << ": " << (*_i)->type);
        }
    }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "basler_camera");
  ros::NodeHandle nh("~");

  if(!nh.hasParam("frame_rate"))
  {
      ROS_ERROR("frame_rate param has been removed. Please use AcquisitionFrameRate and remember to set AcquisitionFrameRateEnable=True");
      nh.deleteParam("frame_rate");
  }

  string camera_info_url;
  if(!nh.getParam("camera_info_url", camera_info_url))
    camera_info_url = "";

  string frame_id;
  if(!nh.getParam("frame_id", frame_id))
    frame_id = "";

  std::string serial_number;
  if(!nh.getParam("serial_number", serial_number))
    serial_number = "";

  std::string camera_name = nh.getNamespace();

  int exitCode = 0;

  camera_info_manager::CameraInfoManager cinfo_manager_(nh, camera_name);

  // Automatically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
  // is initialized during the lifetime of this object.
  Pylon::PylonAutoInitTerm autoInitTerm;
  CGrabResultPtr ptrGrabResult;

  try
  {
    CTlFactory& tlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    if (tlFactory.EnumerateDevices(devices) == 0)
    {
      throw RUNTIME_EXCEPTION("No camera present.");
    }

    if (serial_number == "") {
      // Create an instant camera object for the camera device found first.
      camera.Attach(CTlFactory::GetInstance().CreateFirstDevice());
    } else {
      // Look up the camera by its serial number
      for (size_t i=0; i<devices.size(); i++) {
        if (devices[i].GetSerialNumber().c_str() == serial_number) {
          camera.Attach(tlFactory.CreateDevice(devices[i]));
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

    camera.RegisterImageEventHandler(new ImagePublisher(nh, cinfo, frame_id), RegistrationMode_Append, Cleanup_Delete);
    camera.RegisterConfiguration(new CAcquireContinuousConfiguration , RegistrationMode_ReplaceAll, Cleanup_Delete);

    camera.Open();

    handle_parameters(camera);

    // Set the pixel format to RGB8 if available.
    INodeMap& nodemap = camera.GetNodeMap();
    CEnumerationPtr pixelFormat(nodemap.GetNode("PixelFormat"));
    String_t oldPixelFormat = pixelFormat->ToString();
    if (IsAvailable(pixelFormat->GetEntryByName("RGB8")))
    {
      pixelFormat->FromString("RGB8");
    }

  }
  catch (GenICam::GenericException &e)
  {
    ROS_ERROR_STREAM ("An exception occurred during setup: " << e.GetDescription());
    exitCode = 1;
    return exitCode;
  }

  dynamic_reconfigure::Server<basler_camera::CameraConfig> server;
  server.setCallback(configure_callback);

  while( ros::ok() )
  {
    try
    {
      if(!camera.IsGrabbing())
      {
        camera.StartGrabbing();
      }
    }
    catch (GenICam::GenericException &e)
    {
      ROS_ERROR_STREAM("An exception occurred trying to start grabbing: " << e.GetDescription());
      return 2;
    }

    while (camera.IsGrabbing() && ros::ok())
    {
      ros::spinOnce();
      try
      {
        camera.RetrieveResult(1, ptrGrabResult, TimeoutHandling_Return);
      }
      catch (GenICam::GenericException &e)
      {
        ROS_ERROR_STREAM("An exception occurred during operation: " << e.GetDescription());
        break;
      }
    }
  }
}
