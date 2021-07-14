# Rasendriya  
![RasendriyaUAV](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/docs/rasendriya.jpg)
Rasendriya is a lightweight fixed wing UAV from AUAV UI for Tübitak International Unmanned Aerial Vehicle (UAV) Teknofest Competition.

For Tubitak 2021, Rasendriya has two missions: Flying in oval track and loiter in one point with small radius, and flying in oval track for 3 laps while dropping two payloads in a designated dropzone. The designated dropzone must be found autonomously by using computer vision system.

## Mission Details
### Mission 1
Mission 1 consist of flying in oval track and loiter in pole with small radius. No computer vision required.
### Mission 2
Mission 2 also similar to mission 1. But instead of flying in oval track, the plane must track the dropzone and drop the payload 2 times.
## Mission Flow and Algorithm
More to write later

## Setup
### Prerequisites
- ROS Noetic Ninjemys

#### Companion Computer
- Ubuntu MATE 20.04 LTS for Odroid XU4

#### Desktop
- Ubuntu 20.04 LTS Focal Fossa

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
MAVROS:  
`roslaunch mavros apm.launch`  
Mission 1:
`roslaunch rasendriya mission1.launch`  
Mission 2:
`roslaunch rasendriya mission2.launch`  

### ngrok
Start ngrok service  
`./ngrok tcp 22`  
ssh via ngrok tunneling  
`ssh odroid@<ip_ngrok> -p <port_ngrok>`  

#### vnc Remote Desktop  
Run vnc server  
`sudo x11vnc -display :0 -auth guess`  


