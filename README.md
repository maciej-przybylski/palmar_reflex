# palmar_reflex
Enables Baxter to close its gripper when break beams detect a break, which mimics the palmar grasp reflex in children<br>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=aQBKxrKN0UA
" target="_blank"><img src="http://img.youtube.com/vi/aQBKxrKN0UA/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="640" height="480" border="10" /></a>

1. Install Arduino: https://www.arduino.cc/en/Main/Software, untar it in your directory.

2. Copy this package `palmar_reflex` to your ros workspace, e.g. row_ws/src, then build the package.

3. Connect the break beam sensors (attached on Baxter's girppers) to the arduino board. Details about the sensors that we are using is available at https://learn.adafruit.com/ir-breakbeam-sensors.

4. Plug Arduino board to your pc via the USB cable, and start arduino by 
  ```
  $ cd ARDUINO_DIRECTORY
  $ ./arduino 
  ```
  open file `palmar_reflex/break_beam.ino` in arduino window, and click "upload" button. This will get arduino running and output either "Unbrok" or "Broken" depending on the break beam status. <br>
  
  Make sure that under Menu/Tools/, 
  * Board is selected as the version of the connected arduino board, e.g. "Arduino/Genuino Uno" in our case
  * Port is automatically selected. 
  * Add read&write permission to the serial port that arduino is using. For example, if "/dev/ttyACM0" is the one displayed under Arduino menu, then 

    ```
    $ sudo chmod a+rw /dev/ttyACM0
    ```
    To permanently change the permission of the serial port, please refer to this thread: http://askubuntu.com/questions/58119/changing-permissions-on-serial-port.
  * Open Menu/Tools/Serial Monitor, and check whether there are data coming through the serial port, with your hand waving in between Baxter's grippers, thus chaning the break beam status.
  
5. Start Baxter, and calibrate the left gripper first 

  ```
  $ rosrun baxter_example gripper_keyboard.py
  ```
  then press "c".
  
6. Launch ros node that publishes the break beam to topic "/break_beam" status by reading data from the serial port,

  ```
  $ rosrun break_beam break_beam_publisher
  ```

7. In another terminal launch ros node that listens to topic "/break_beam" and commands the gripper to close when "Broken" is detected,
  
  ```
  $ rosrun break_beam break_beam_listener
  ```
  
  
  
