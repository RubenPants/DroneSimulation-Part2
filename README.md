
# DroneSimulation-Part2 - Overview

A simulation of a drone which will fly in a three dimensional virtual testbed. An array of unit cubes will be loaded inside the virtual testbed, the main
objective of the drone is to collect all cubes as fast as possible (no order specified). This project will build further upon the project conventionally
named _DroneSimulation-Part1_ (https://github.com/RubenPants/DroneSimulation-Part1).



# Run the Project

To run the project, and thus open the testbed's Graphical User Interface, go to 'Virtual Testbed' -> 'src' -> 'main' -> 'MainLoop' and run 'MainLoop' as a 
Java Application. When starting the project, two windows will pop up: the virtual testbed itself and a window where you can toggle the option to control the
drone using your phone (Android only). A description on how to connect your phone with the drone is given in a later subsection (_Control Drone using Phone_).



# Virtual Testbed

The _virtual testbed_ will provide a clear representation of the drone and its actions. Inside the GUI there are multiple windows which will add a corresponding
functionality to the project. A quick overview of these windows are:  
* __File__ - Add a custom path to the testbed  
* __Settings__ - Change the drone or the testbed settings
* __Run__ - Manage the flow of the program (start, reset, run tests, ...)
* __Window__ - Change the drone-view
* __Inputs__ - Standard information and settings of the testbed and drone
* __Configuration__ - Change the drone's configuration
* __Configure Path__ - Configure a given or custom path to load in the testbed



# Autopilot

In this version (the second one) the autopilot will do more sophisticated calculations to perform certain actions in comparison with its first version. The
camera of the drone will now be used to locate the cubes inside the testbed (saved as coordinates). An algorithm will then calculate the best possible path
to collect all cubes the drone has encountered in as less time as possible. The drone will fly towards the cubes in a Dubins-like path 
(https://en.wikipedia.org/wiki/Dubins_path). The drone will perform its actions using a PD-controller (Proportional and Derivative). A use of a PID-controller
(I for Integral) was not needed since the goal of the drone was to get at all times a zero error based on its current position and the requested position on
the calculated path.

In this version, the drone will start on the ground and first has to take off before it will be able to collect cubes. When all cubes are collected the drone
will land. The time (that the drone must try to minimize) will start running from when the program is started (and thus when the drone is steady on the
ground) and will end when all cubes are collected and the drone is back at its starting position.



# Automatic Running Tests

It is possible to test the drone on randomly generated paths. Go inside the testbed's GUI to 'Run' -> 'Run tests', a new window will pop up. Within the _testing
frame_ you can manage the tests: change the amount of tests, change the speed of the drone during the tests, change the maximum time the drone has to accomplish
one testing path, and more.



# Changing the AutoPilot

When changes are made within the _AutoPilot_, you have to export the whole _AutoPilot_ file as a jar and place this jar in the _Virtual Testbed_ file on the
following location: 'Virtual Testbed' -> 'lib' -> 'jar'. You __must__ name this jar 'autopilot.jar' otherwise the testbed will not recognize the jar. At the
moment there is no functionality to toggle between multiple jars, and thus it will not be possible to test or compare two or more autopilots at the same time.



# Control Drone using Phone

In this version of the project, it will be able to control the drone using your Android phone with the help of an UDP server that is hosted within the autopilot. To do so, you first need to do several things:  
* Install the Android app on the Google play-store (https://play.google.com/store/apps/details?id=com.broekxruben.teamsaffier)  
* Use your phone as a mobile hotspot, this is needed to provide a fluent connection between your phone and the drone  
* Configure your PC's IP address within the app (open the terminal on your PC and type _ipconfig_ and search for IPv4 or IPv6)  
* Run the Java Application (MainLoop) and toggle the popup window to active (_toggle connection with phone_)  
* Start the app: go to 'Run' -> 'Start Simulation'

If everything went right, the drone should stay steady on the ground. If the drone starts to slowly take of (and does not respond to your actions), you did not
successfully connect your phone with the drone.



# History of the Project

This project is the seccond part of a larger whole:
* __Part1__ - Fly in the testbed.  
Link: https://github.com/RubenPants/DroneSimulation-Part1  
* __Part2__ - Take off, fly, land and taxi in the testbed. Control drone with phone.  
Link: https://github.com/RubenPants/DroneSimulation-Part2  
* __Part3__ - Example of a worst case of the package-distributing-system of _Part4_ where all the drones try to land at the same airport.  
Link: https://github.com/RubenPants/DroneSimulation-Part3  
* __Part4__ - A package-distributing-system where multiple drones must work together to distributed packages in a virtual environment.  
Link: https://github.com/RubenPants/DroneSimulation-Part4  
