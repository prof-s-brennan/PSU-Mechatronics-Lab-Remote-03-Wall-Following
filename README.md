# PSU-Mechatronics-Lab-Remote-03-Wall-Following
This is repository for the lab files to implement simulated robot wall following in MATLAB. It demonsrates a simple robot kinematic simulation, drawing utility, and compartimentalization of code.
See the GitHub wiki page for more details, at: 
https://github.com/prof-s-brennan/PSU-Mechatronics-Lab-Remote-03-Wall-Following/wiki/Remote-Lab-Material-to-Simulate-a-Wall-Following-Robot

Installation: Copy the files into a working directory of MATLAB. The main code is within script_simRobotWallFollowing.m.

Usage: the main script will run and animates a robot. The animation is slow, but can be sped up by changing the parameter within the main script. The core kinematics and drawing functions are put into a separate code that is meant to be compiled into a .p file, to hide functionality from students when sharing.

Contributing: Nothing formal is set up yet for contributions. If you are interested, just email sbrennan@psu.edu

Credits: Code is written by sbrennan@psu.edu

License: 
MIT License

Copyright (c) 2020 Sean Brennan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
