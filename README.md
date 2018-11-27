# uavEE
<b>uavEE</b> is a modular Emulation Environment for rapid development and testing of Unmanned Aerial Vehicles (UAVs). Its main functionality, structure, and the integration of a propulsion power model are described in [1]. 


<b>Dependencies</b>

uavEE is a collection of ROS-based packages. The standard ROS version used for uavEE is ROS-Melodic. uavEE depends on the Core of <a href="https://github.com/theilem/uavAP">uavAP</a>, therefore, uavAP has to be installed first. The ground station depends on Qt5 (Components: Core, Gui, OpenGL, Svg, PrintSupport).


<b>Setup</b>

When all the dependencies are installed an execution of catkin_make in the main package directory suffices to compile the nodes (Due to circular message dependencies the first run of catkin_make can fail, a second run usually solves the issue). To install the XPlanePlugin first create a folder "{xplane_dir}/Resources/plugins/uavee/64/". Then copy the library from "{uavEE_path}/devel/lib/liblin.xpl.so" to the created folder, renaming it to "lin.xpl".


<b>Real-Flight Components</b>

RadioComm: 
This node receives packets through a serial interface and distributes them to the ros network. Some of the messages received from the autopilot have a dedicated translation into ros messages (e.g. SensorData), others are forwarded as serialized messages and interpreted by the recipient (e.g. Trajectory). RadioComm adds services to allow the communication to the autopilot (again some with dedicated messages and others serialized).

GroundStation:
The GroundStation GUI displays the status of the autopilot in various ways. Additionally, it is used as a UI for calling the services of RadioCommm to communicate to the autopilot (e.g. Tuning PID parameters online). 


<b>Emulation Components</b>

Simulation Interfaces:
To connect the emulation environment to a flight simulator a simulation interface node is needed. The main interface that is used is the XPlaneInterface. This node is not directly launched by a launch file, but rather created from a plugin in <a href="https://www.x-plane.com/">X-Plane 11</a>. The installation of this plugin was described before. If other flight simulators are to be used, a custom interface node has to be programmed. The interface node will need to publish SensorData and subscribe on Actuation.

AutopilotInterface:
This node is responsible to supply the autopilot with the SensorData coming from a SimulationInterface and retrieve the actuation command. The implemented interface communicates with uavAP running on the same system via a fake serial port (<a href="https://github.com/freemed/tty0tty">tty0tty</a>). Other options for the autopilot interface are to parse a log file and feed the sensor data directly into the autopilot, as well as connecting to an autopilot running on an embedded board via a serial connection.


<b>Additional Components</b>

PowerModeling:
This node brings power awareness into the environment, implementing the power models in [1] and [2].

PanTiltHandler:
An experimental implementation of a controller for a PanTilt system aiming an antenna or camera on the flying aircraft.


[1] Theile, M., Dantsker, O. D., Nai, R., and Caccamo, M., “uavEE: A Modular, Power-Aware Emulation Environment for Rapid Prototyping and Testing of UAVs,” IEEE International Conference on Embedded and Real-Time Computing Systems and Applications, Hakodate, Japan, Aug. 2018.

[2] O. D. Dantsker, M. Theile, and M. Caccamo, “A high-fidelity, low-order propulsion power model for fixed-wing electric unmanned aircraft,” AIAA/IEEE Electric Aircraft Technologies Symposium, Jul. 2018.
