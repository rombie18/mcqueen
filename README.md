# McQueen

![Lightning McQueen](https://media.tenor.com/UHy0nbKkmSYAAAAd/lightning-mcqueen-cars-movie.gif)

> “Ka-chow!” - Lightning Mcqueen

A piece of python software to control an autonomous rc car with camera vision and odometry.

## Working principle

The program will start by initialising the actuators (motor + servo), PID controllers and the controller. Next all other sensors or functions will be initialised and run in parallel as threads. Once all threads are initialised, the program will release all threads to start processing or collecting/writing data. The data is send over a pipe that will interconnect the threads with the main program.

## Installation

Please follow the steps below to install the software on your Jetson Nano.

1. Clone this repository to your user directory:  
   `cd ~`  
   `git clone https://github.com/rombie18/mcqueen`
2. Run the installation script[^1] (with root permissions):  
   `cd mcqueen/Software/install`  
   `sudo -H bash ./install.sh`
3. All done!

## Running the software

As configured by the installation script, the software should automatically start running when the Jetson Nano boots up. If for any reason you would want the program to stop, you can gracefully terminate the process by executing `sudo systemctl stop mcqueen`. You can manually start the program by running the main python script `python drive.py`.

[^1]: _The installation script will install libraries and configure a services automatically for you. This includes updating libraries, installing circuitpython, installing and enabling service,..._
