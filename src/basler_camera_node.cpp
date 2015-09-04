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

  int frame_rate;
  if( !priv_nh.getParam("frame_rate", frame_rate) )
    frame_rate = 20.0;

  // The exit code of the sample application.
  int exitCode = 0;

  // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
  // is initialized during the lifetime of this object.
  Pylon::PylonAutoInitTerm autoInitTerm;
  CGrabResultPtr ptrGrabResult;

  try
  {
    // Create an instant camera object for the camera device found first.
    CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());

    // Register the standard configuration event handler for enabling software triggering.
    // The software trigger configuration handler replaces the default configuration
    // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
    camera.RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);

    // For demonstration purposes only, add a sample configuration event handler to print out information
    // about camera use.
    //camera.RegisterConfiguration( new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete);

    // The image event printer serves as sample image processing.
    // When using the grab loop thread provided by the Instant Camera object, an image event handler processing the grab
    // results must be created and registered.
    camera.RegisterImageEventHandler( new ImagePublisher(nh), RegistrationMode_Append, Cleanup_Delete);

    // Start the grabbing using the grab loop thread, by setting the grabLoopType parameter
    // to GrabLoop_ProvidedByInstantCamera. The grab results are delivered to the image event handlers.
    // The GrabStrategy_OneByOne default grab strategy is used.
    camera.StartGrabbing( GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
    //camera.StartGrabbing( GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);

    ros::Rate loop_rate(frame_rate);
    while (nh.ok()) {
      camera.ExecuteSoftwareTrigger();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch (GenICam::GenericException &e)
  {
    ROS_ERROR_STREAM ( "An exception occurred." << e.GetDescription());
    exitCode = 1;
  }
  return exitCode;
}

