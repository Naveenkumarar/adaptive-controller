# F1/10 human racing

### Get Started
1. Clone repo
```
mkdir catkin_ws
git clone https://github.com/resilient-autonomous-systems-lab/F1-10-human-racing.git src
cd src
git submodule update --init
```
2. Install dependencies:
- joy
```
sudo apt update
sudo apt install ros-$ROS_DISTRO-joy*
```
- pacmod
```
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-pacmod
```

3. Build and run packages
```
cd ..
catkin build
source ~/catkin_ws/devel/setup.bash
roslaunch global_launch racing.launch
```

### git token for raslab
- username: fsudssAI
- passcode: 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

### Software log
The communication and software framework has been steuped
- subscribe joy commands from racing cockpit
- simply scaling steering and throttle commands
- Publish commands to MicroNole
- Create RTSP server for MicroNole's front camera
- Stream videos to jetson <br>

### Alpha test
Suggestions from Alpha testers:
 1. steering torque feedback, auto steering back to middle 
 2. trottle command has dead zone due to the friction of the tire, try to find the minimal throttle which can move the car physically in order to remove the dead zone.
 3. Display the speed value and other position infromation of the car on cockpit's monitor.
 4. model the real deceleration situation
 5. add the middle lane tape   (Done)
 6. get two another small monitors to show the right and left camera view,
 7. turn on back camera view when move backward

### User guide
1.	If you don’t have VNC viewer, please download any VNC viewer on your computer 
(For example: RealVNC, https://www.realvnc.com/en/connect/download/viewer/ )

2.	Connect your computer to wifi “RASLab”:

> Passcode: r@$Lab21

3.	Open your VNC viewer, access Micronole computer and Racing cockpit computer <br>

MicroNole: 

> IP: 192.168.2.13

> Passcode: nvidia 
 
 Racing cockpit: 

> IP: 192.168.2.19

> Passcode: nvidia <br>

**Bug tips:** <br>
If you cannot access those computer through VNC, it is probably the IP problem. Connect those two computers to a monitor and keyboard and mouse, open terminal, type “ifconfig”, it will show the IP address:<br>
 <img src="https://user-images.githubusercontent.com/36635562/163507493-f3282236-4edd-4212-83d0-7ac788d4be8d.png" width="500" />


4.	Open terminal in MicroNole computer, and type: <br>
`sudo -s` <br>
`nvidia` <br>
`roslaunch qcar racing.launch` <br>
<img src="https://user-images.githubusercontent.com/36635562/163508212-e177e85a-ad35-4ec8-823b-0d4306fb8cd8.png" width="500" /> 
<img src="https://user-images.githubusercontent.com/36635562/163508245-f26b8dfc-3316-4abd-8ea7-fb473e0ad9f1.png" width="500" />

5.	Open terminal in Racing cockpit computer, and type: <br>
`sudo -s` <br>
`nvidia` <br>
`roslaunch global_launch racing.launch` <br>
<img src="https://user-images.githubusercontent.com/36635562/163508312-832cb84a-f877-41f5-afd0-3614ed5c725f.png" width="500" />

Once see the camera view, close it by clicking “x” to close the image window, then it will show a bigger window:
<img src="https://user-images.githubusercontent.com/36635562/163508336-7855d22b-2ba1-4e47-bc8b-9af9747d87db.png" width="500" />

**Bug tips:**
If show “ unable to open camera” like below: <br>
<img src="https://user-images.githubusercontent.com/36635562/163508431-5a6a50df-9f29-40f6-9ab5-0d3312f647e3.png" width="500" />

Use “ctrl+c” to stop the process, and launch it again

6.	Now you are ready to go, have fun!



