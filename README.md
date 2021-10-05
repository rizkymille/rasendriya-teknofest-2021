# Rasendriya
!['Rasendriya Logo'](https://github.com/rizkymille/rasendriya-teknofest-2021/blob/main/docs/logo.png)  
Rasendriya is a lightweight fixed wing UAV from AUAV UI for Tübitak International Unmanned Aerial Vehicle (UAV) Teknofest Competition in Bursa, Turkey.

For Teknofest 2021, Rasendriya has two missions: Flying in oval track and loiter in one point with small radius, and flying in oval track for 3 laps while dropping two payloads in a designated dropzone. The designated dropzone must be found autonomously by using computer vision system.

## Mission Details
### Mission 1
Mission 1 consist of flying in oval track and loiter in pole with small radius.
### Mission 2
Mission 2 also similar to mission 1. But instead of flying in oval track, the plane must track the dropzone and drop the payload 2 times.

## Short branch explanation
There's so many branch because I'm still learning programming when creating this ROS program, so the program always got updated to be more efficent and sophisticated. I don't want to delete the older version because it has some good programming example. To avoid confusion, I will explain what's the difference between every branch:
#### "main" branch
This is the program I originally want to use for the flight testing and even to the final competition. Feature:
- Waypoint manipulator API  
Manipulate waypoint in pixhawk when flying using this API. It feels like "automated GCS". It can swap, erase, and insert waypoints
- Write waypoint API  
Create servo dropping waypoint autonomously when flying.
#### "simpler" branch
This is the program I thought in day 5 of the competition, because how prone to fail image processing can be. Feature:
- Waypoint rewrite  
Rewrite some waypoint values (like latitude and longitude). If the image processing fails to get the coordinate, it still uses default coordinate GCS write before launch. 
#### "azure" branch
This is the branch I created when Microsoft agree to be AUAV sponsor. Feature:
- Azure IoT Hub  
Sending ROS data directly from pixhawk to Azure IoT Hub in real-time. Works by dumping data into JSON format first and then send it into Azure cloud.
#### "complex" branch
This is the first rasendriya program I created because I don't know how ROS/C++ works so I just imitate and modify AUAV VTOL Athena ROS program (you can look at github.com/vtol-auav-ui/athena, forked from Kevin Yosral). Turns out I don't need to create class because the mission program isn't as complex as it seems. Feature:  
- C++ Class  
Good example to learn OOP :)


## How This Program Works (Algorithm)
The ROS Rasendriya package acts as object detector and 'automated GCS'. Flowchart of this program can be seen below:

Overall, this program can be splitted as these parts:
### Object Detection
This package detects dropzone by two main principles: color filtering, and shape detecting. The vision_dropzone.py program will turn red color into white and turn other color into black. Then, the program will detect circle shape by using Circle Hough Transform. The result of this process is pixel coordinate of circle center.
### Camera Projection
After circle center coordinate is found, it must be converted to real-life unit, like meter. This is possible by using camera projection formula, which uses focal length. That's why it's very recommended to calibrate your camera first to get the intrinsics value.
### Coordinate Transformation
The real life coordinate value still not enough because this UAV use GPS, which uses WGS84 system. To transform this coordinate system, haversine law is used.
### Waypoint Planning and Pushing
After all required data is completed, then the package will send waypoints command to pixhawk.  
If dropzone coordinate is found, then the program will command pixhawk to repeat mission and send waypoint commands to lower height at certain coordinate and release the payload servos. 


## Setup
### Prerequisites
- ROS Noetic Ninjemys

#### Companion Computer
- Ubuntu MATE 20.04 LTS for Raspberry Pi 4B

#### Libraries
- OpenCV 4

#### ROS Packages
- MAVROS

### Headless Setup
#### SSH
Check SSH running  
  `sudo systemctl status sshd`  
Start SSH  
  `sudo systemctl enable ssh`  
  `sudo systemctl start ssh`  

#### ngrok
Install ngrok  
`sudo snap install ngrok`  
`ngrok authtoken <your_auth_token>`

#### vnc Remote Desktop  
Install vnc server  
`sudo apt update`  
`sudo apt install x11vnc`  
Run vnc server  
`sudo x11vnc -display :0 -auth guess`  

## Launching Headless
### SSH  
Connect your ground control station to vehicle SBC via ssh  
`ssh device_name@ip_address`  
For odroid:  
`ssh odroid@10.107.213.213`  

### ROS
`roslaunch rasendriya mission2.launch`  

### ngrok
Start ngrok service  
`ngrok tcp 22`  
ssh via ngrok tunneling  
`ssh odroid@<ip_ngrok> -p <port_ngrok>`  

#### vnc Remote Desktop  
Run vnc server  
`sudo x11vnc -display :0 -auth guess`  

## ROS configure
Set devel path in bashrc first  
`sudo pluma ~/.bashrc` or `sudo gedit ~/.bashrc`  
then add this in the last line  
`source /home/pi/catkin_ws/devel/setup.bash`
