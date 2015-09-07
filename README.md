Basler camera interface with ROS
========================

These are the steps needed to publish Basler camera images to ROS:

1. Get the Pylon 4 Camera Software Suite from the Basler website:

   http://www.baslerweb.com/en/support/downloads/software-downloads?type=28&series=0&model=0

2. Follow these instructions (also available in the install file):

  a. Change to the directory which contains this INSTALL file, e.g.: 
  ```bash
  cd ~/pylon-4.0.0-x86
 ```
  b. Extract the corresponding SDK into /opt
  ```bash
  sudo tar -C /opt -xzf pylonSDK*.tar.gz
  ```
  c. Install udev-rules to set up permissions for basler USB cameras
  ```bash
  ./setup-usb.sh
  ```
       
  d. Unplug and replug all USB cameras to get the udev rules applied.
  
  e. Execute /opt/pylon4/bin/PylonViewerApp to test your cameras.
  
3. Setup your environment for pylon to find the necessary dependencies:

   ```bash
   source /opt/pylon4/bin/pylon-setup-env.sh /opt/pylon4
   ```

4. Build the package using catkin_make

5. Launch the basler_Camera node with:

   ```bash
   roslaunch basler_camera basler_camera.launch
   ```

   The frame rate could be modified using the ```frame_rate``` parameter.

6. The image can be visualized using image_view:

   ```bash
   rosrun image_view image_view image:=/camera/image_raw
   ```



