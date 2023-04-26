"""
This python file is the main entry point of the robot.
It will be executed when the robot starts up, and will be attempted to restart if it exits.

Please run other files or do some simple logic below to start the appropriate program(s).
"""

with open("main/drive.py") as file:
    exec(file.read())