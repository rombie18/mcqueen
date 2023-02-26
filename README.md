# McQueen

![Lightning McQueen](https://media.tenor.com/UHy0nbKkmSYAAAAd/lightning-mcqueen-cars-movie.gif)

> “Ka-chow!” - Lightning Mcqueen

A piece of python software to control an autonomous rc car with camera vision and odometry.

## Task list

- [x] Start hotspot on boot
- [ ] Auto pull repo on internet available
- [ ] Reorganise program structure
- [ ] Add support for control with PS4 remote
- [ ] Apply basic camera vision
- [ ] Add encoder measurement
- [ ] Think out algorithm for lane following/control
- [ ] Optimize and debug
- [ ] (Combine camera/imu/encoder measurements for position estimation)
---

## Installation

Please follow the steps below to install the software on your Jetson Nano.

1. Clone this gif to your user directory:  
   `cd ~`  
   `git clone https://github.com/rombie18/mcqueen`
2. Run the installation script[^1] (with root permissions):  
   `cd mcqueen`  
   `sudo ./install.sh`
3. All done!

## Running the software

As configured by the installation script, the software should automatically start running when the Jetson Nano boots up. If for any reason you would want the program to stop, you can gracefully terminate the process by executing `pkill -f main_program.py`. You can manually start the program by running the main python script `python main_program.py`.

[^1]: _The installation script will configure some services automatically for you. This includes setting up VNC-access, automatically starting the main program on boot, starting a wifi hotspot on boot, automatically pulling the repository when internet is available,..._
