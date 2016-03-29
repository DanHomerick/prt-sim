# Overview #

A vehicle controller is a standalone program that communicates with the sim via a TCP connection. Messages are sent using protocol buffers for the wire-format. All communications between the sim and controller(s) take place while the simulation is paused. The sim pauses after every event, and does not unpause until every controller has indicated that it is ready to resume. Controllers can use the `CtrlSetNotify___` messages to have the sim pause at specified times or when a vehicle reaches a specified position.

### Messages ###
The definitive documentation for event messages is found in the [api.proto](http://code.google.com/p/prt-sim/source/browse/trunk/pyprt/shared/api.proto) file.

Each message begins with a fixed-length header, comprised of the following:
  * message separator
  * message type
  * message id
  * simulation time
  * message size

The header is followed by a message in [Protocol Buffers](http://code.google.com/p/protobuf/) format (AKA protobuf). Protocol Buffers are "a language-neutral, platform-neutral, extensible way of serializing structured data for use in communications protocols, data storage, and more." -- see that project's [user docs](http://code.google.com/apis/protocolbuffers/docs/overview.html) for a more complete introduction. The messages are defined and documented in the api.proto file.

Note that the protobuf messages are not self-describing. The message type and length must be determined from the information in the fixed-length header. The sim expects that the messages it receives will have a header in the same format.

### Communication ###
During communications setup, the sim plays the server role. The sim opens a TCP port (specified in sim's config settings) and waits for the specified number of controllers to make connections to it. Once the connections have been established, the sim sends a `SimGreeting` message to all controllers.

When the simulation has been prompted to begin by the user, the sim sends a `SimStart` message(s) and immediately pauses. Simulation time does not begin passing until all controllers respond with a `CtrlResume` message. Once all controllers have responded, the sim unpauses and sim-time passes. While the sim is running, the controllers cannot initiate communication. Once an event occurs the simulation is paused and message(s) announcing the event are sent to the controller(s). As before, the sim remains paused until all controllers send a `CtrlResume`.

Controllers can use `CtrlSetNotify____` messages to force the sim to pause at a specified time or when a vehicle reaches a specified position. When the condition is reached, a `SimNotify____` event will trigger, and a corresponding message will be sent.

The `CtrlResume` message requires that the controller include the unique message ID from the most recent message it has seen from the simulator. This allows the sim to know which message the controller is responding too -- if the message ID does not correspond with the most recent message send by the sim, then the `CtrlResume` is ignored and the sim remains paused. This allows the controller to send a `CtrlResume` in response to every message from the sim, while knowing that if additional messages are forthcoming the controller will be given a chance to respond to them before the simulation resumes.

At the end of the simulation, the simulator will send a `SimEnd` message.

### Trajectories / Cubic Splines ###
Vehicles' trajectories are controlled via cubic splines which specify position, velocity, acceleration, and jerk over some time span. Note that in order for a controller to generate a control spline, it must have some estimate of the initial position, velocity, and acceleration. Only the jerk parameter does not require the controller to have an accurate estimate of the vehicle's initial state. When a sim receives a spline, the sim uses the spline's jerk and time values and the sim's true initial position, velocity and acceleration values to recreate the spline. Discrepancies between the controller's spline and the recreated spline are logged. Only using the jerk prevents the controller from accidentally "warping" the vehicle from one location to another.

<a href='http://img52.imageshack.us/i/cubicspline.png/' title='Cubic Spline'><img src='http://img52.imageshack.us/img52/593/cubicspline.png' border='0' /></a>

### Planned Features ###
  * Currently, every controller receives a copy of every event message. I have a more selective publish/subscribe system planned which will allow a controller to only subscribe to some events, or just to events relating to particular vehicles.
  * As is, communications between controllers are not facilitated by the sim. I intend to add a system wherein controllers communicate with each other through the simulator. This will allow the simulator to add simulated latency to their communications.
  * May add support for UDP in addition to TCP.