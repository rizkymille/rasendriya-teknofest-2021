# Rasendriya  
![RasendriyaUAV](https://github.com/rizkymille/rasendriya-auav-ui/blob/main/docs/rasendriya.jpg)
Rasendriya is a lightweight fixed wing UAV from AUAV UI for Tübitak International Unmanned Aerial Vehicle (UAV) Teknofest Competition.

For Tubitak 2021, Rasendriya has two missions: Flying in oval track and loiter in one point with small radius, and flying in oval track for 3 laps while dropping two payloads in a designated dropzone. The designated dropzone must be found autonomously by using computer vision system.

## Mission Details
More to write

## Mission Flow and Algorithm

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
- Imutils

### Odroid Headless Setup
#### SSH
Install SSH at your local SBC  
`sudo apt install openssh-server`  
Check SSH running  
`sudo systemctl status sshd`  
`sudo systemctl enable ssh`  
`sudo systemctl start ssh`  

#### Ngrok

#### vnc Remote Desktop
Install vnc server:
`sudo apt update`
`sudo apt install x11vnc`
Run vnc server:
`x11vnc -display :0 -auth guess`

## Accessing Odroid Headless
#### SSH
Connect your ground control station to vehicle SBC via ssh
`ssh device_name@ip_address`
For odroid:
`ssh odroid@10.107.213.213`

