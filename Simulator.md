# Overview #
The core sim is responsible for tracking the world state and for updating the state based on the passage of time, input from the vehicle controller(s), and time-based events. It is a "micro-simulator" and models each vehicle's and passenger's actions explicitly, rather than using statistical methods.

### Level of Detail ###
The sim is not a particularly detailed simulation of vehicle or guideway physics. In designing a Personal Rapid Transit system, there are many different choices that a system implementor must make regarding a vehicle's sensors, propulsion and suspension as well as properties of the guideway. For this project, many of those details are abstracted away. The focus is instead on collecting statistics for passenger waiting times, trip times, average velocities, fraction of time or distance that a vehicle spends travelling empty, etc.

### Events ###
The sim is event based and sends messages to the controller(s) as they occur. Examples of events are:
  * A vehicle's nose reaches the end of a track segment.
  * A vehicle's tail clears a track segment.
  * A vehicle's nos reaches a specified position.
  * A specified time is reached.
  * The distance between two vehicles falls below a specified limit.
  * A collision occurs.
  * A passenger is created at a station.

For a full list of event messages and their documentation see the [api.proto](http://code.google.com/p/prt-sim/source/browse/trunk/pyprt/shared/api.proto) file in the source tree.

### Passengers ###
The sim spawns passengers based on data from a passenger file. The passenger file specifies origin and destination stations, spawn time, loading and unloading times, mass, and other values for each passenger. Passengers' positions within a station are not modeled explicitly, beyond a FIFO queue storing them in order of their spawn times.

The scripts directory contains a Python [script](http://code.google.com/p/prt-sim/source/browse/trunk/pyprt/scripts/make_passengers_file.py) that can be used to generate a passenger file from some parameters.

### Time ###
It is useful if the sim can run at faster than real time, and if it can tolerate the vehicle controller(s) taking time to compute. After all, a single CPU may be running many vehicle controller(s) and have the burden of running other software too. To allow for this, the sim has not been designed to run in a true real-time fashion. Instead, when an event occurs the simulation is paused and event message(s) are sent out. The controller(s) may then send commands or queries to the sim, which remains paused until all controllers send a 'resume' message.

The speed of simulation may be slowed so that it approximately matches some multiple of wall-clock time when watching a live visualization.

### Coordinate Frames ###
Each track segment (a straight or curved piece of track) has its own coordinate frame. While vehicle positions are stored internally as just an offset from the vehicle's initial position, vehicle positions are communicated to controllers in the form of a unique track segment id and the distance from the beginning of the segment.

### Vehicle Control ###
Vehicle trajectories are controlled by cubic splines which specify position, velocity, acceleration, and jerk over some time span. Vehicle paths are specified as either a sequence of track segment ids (assumes track-switching hardware is mounted on the vehicle) or as commands to toggle a track switch from one side to the other. See [Sim/Controller Communication](SimControllerComm.md) and [api.proto](http://code.google.com/p/prt-sim/source/browse/trunk/pyprt/shared/api.proto) for information about other commands.

See planned features for a note about quadratic splines.

### Configuration File ###
Each simulation scenario should have a configuration file dedicated to it. The config file is in [INI](http://en.wikipedia.org/wiki/INI_file) file format. The config file specifies paths to the track layout files, log files, gives startup commands for the vehicle controller(s), and specifies various other settings for the sim.

# See Also #
[Sim/Controller Communications](SimControllerComm.md)

# Planned Features #
  * In its current form, the sim is entirely deterministic and noise free. Eventually one will be able to specify a noise model that the sim will use to perturb the position, velocity, and acceleration data before it is sent to a vehicle's controller.
  * Event and status messages are currently passed to the controller(s) without any simulated latency. When implemented, event messages can be delayed by some (possibly random) amount of time before being sent to the controller(s).
  * Using cubic splines results in finite jerk values and in continuous acceleration profiles (no discontinuities). In general, those are nice traits, but the trade-off is that they are more complicated to work with than quadratic splines. A planned feature is to allow the sim to be configured to accept quadratic splines instead.
  * Configuration files must be manually created with a text editor. Eventually there will be a wizard in the Sim's viz/gui to walk the user through creating a scenario config file.