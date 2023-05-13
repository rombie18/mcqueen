# McQueen

![Lightning McQueen](https://media.tenor.com/UHy0nbKkmSYAAAAd/lightning-mcqueen-cars-movie.gif)

> “Ka-chow!” - Lightning Mcqueen

A piece of python software to control an autonomous rc car with camera vision and odometry.

## Working principle

The program will start by initialising the actuators (motor + servo), PID controllers and the controller. Next all other sensors or functions will be initialised and run in parallel as threads. Once all threads are initialised, the program will release all threads to start processing or collecting/writing data. The data is send over a pipe that will interconnect the threads with the main program.

## Folder structure
~\
└── mcqueen\
&emsp;&emsp;├── Drawings\
&emsp;&emsp;└── Software\
&emsp;&emsp;&emsp;&emsp;├── images\
&emsp;&emsp;&emsp;&emsp;├── install\
&emsp;&emsp;&emsp;&emsp;├── libs\
&emsp;&emsp;&emsp;&emsp;├── main\
&emsp;&emsp;&emsp;&emsp;├── testing\
&emsp;&emsp;&emsp;&emsp;├── videos\
&emsp;&emsp;&emsp;&emsp;└── main.sh

- `~`: Home directory of user, navigate to this with `cd ~` or just `cd`
   - `mcqueen`: The directory name of the repository
      - `Drawings`: Contains all 3D drawings for different plates and mounting brackets
      - `Software`: Contains the software for the robot
         - `images`: Contains various images used for testing and calibrating
         - `install`: Contains the installation script and other important scripts/libraries for the installation of the robot
         - `libs`: Contains custom libraries for the encoder, camera and lane detection
         - `main`: Contains the main programs to drive the robot, to record camera footage and to run lane detection seperately
         - `testing`: Contains various testing scripts to individually test software or hardware components
         - `videos`: Contains recorder footage of the robot in operation (this folder does not get synched to git)
         - **`main.sh`: This is the entry point of the robot. This script will be run when starting the mcqueen service on boot**

## Installation

Please follow the steps below to install the software on your Jetson Nano.

1. Download the Jetson Nano SD card image **version 4.5.1**[^1]
2. Write the `.img` file to the SD card using e.g. Rufus or Etcher 
3. Clone this repository to your user directory:  
   `cd ~`  
   `git clone https://github.com/rombie18/mcqueen`
4. Run the installation script[^2] (with root permissions):  
   `cd mcqueen/Software/install`  
   `sudo -H bash ./install.sh`
5. Install required drivers and additional software for the TIS camera[^3]
6. Configure WiFi hotspot and pair Bluetooth controller
7. Reinstall OpenCV on the Jetson Nano with CUDA support by following [this guide](https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html) (this will enable the use of the GPU instead of CPU for image processing), **this step can take up to 3 hours!**


## Running the software

As configured by the installation script, the software should automatically start running when the Jetson Nano boots up. If for any reason you would want the program to stop, you can gracefully terminate the process by executing `sudo systemctl stop mcqueen`. You can manually start the program by running the main python script in the main directory: `python drive.py`.

> :warning: Make sure to provide power to the encoder when starting the robot, otherwise the Jetson Nano will crash!

## Handy commands
- Disable automatic start program on boot: `sudo systemctl disable mcqueen`
- View status of program: `sudo systemctl status mcqueen`
- Stop running program: `sudo systemctl stop mcqueen`
- Start configuration program for TIS camera: `tcam-capture`
- View CPU/GPU usage, memory and other stats: `jtop`
- Disable Graphical desktop environment: `sudo systemctl set-default multi-user.target`
- Enable Graphical desktop environment: `sudo systemctl set-default graphical.target`
- Temporarily disable Graphical desktop environment (until reboot): `sudo init 3 `


[^1]: https://developer.nvidia.com/embedded/l4t/r32_release_v5.1/r32_release_v5.1/jeston_nano/jetson-nano-jp451-sd-card-image.zip
[^2]: The installation script will install libraries and configure a service automatically for you. This includes updating libraries, installing circuitpython, installing and enabling service,...
[^3]: _https://www.theimagingsource.com/en-us/embedded/mipi-csi-2/36m/dfm36mx296ml/
