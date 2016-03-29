#Vehicle controller for a non-shared vehicle service model.

## Overview ##

This vehicle controller uses the PRT service model, in which vehicles do not adhere to fixed routes or a fixed schedule, and in which vehicles need not be shared with strangers during a trip.

The controller uses a "quasi-synchronous" approach, uses shortest path vehicle routing, has a simple demand-based algorithm for empty vehicle management, and may move vehicles into or out of storage based on the current number of waiting passengers.

The controller only generates reference trajectories for the vehicles to follow. It does not do the lower-level task of feedback control to keep the vehicle on the reference trajectory.

The controller is monolithic, in that there is only one process that controls all of the vehicles in the sim, including all routing and merging tasks.

## Usage ##

Controllers are normally started via the Simulation GUI, with arguments supplied by the scenario's configuration file.

Starting the controller manually can only be done from an SVN checkout, and not from the installed version. Open a command prompt (Windows) or terminal (OS X, Linux) and navigate to the controller's directory. To see what options are available enter:

`python prt_controller --help`

When started, the controller will try to connect to the simulator on the specified port. If the simulator is not already running, the controller will quit with an error message.

## Trajectory Generation ##

## Routing ##

The controller uses a weighted [directed graph](http://en.wikipedia.org/wiki/Directed_graph) representation of the track network in order to route vehicles to their destinations. Each node of the graph is a track segment; each edge represents a connection between two segments. Each edge weight represents the length of the track segment at the edge's tail. Vehicle routes are chosen as the shortest path, as found using a bidirectional variant of [Djikstra's algorithm](http://en.wikipedia.org/wiki/Dijkstra's_algorithm).

In the case that the vehicle with passengers on-board is waved off from entering a station, the vehicle is sent on a loop so that it returns to the station at a later time. To accomplish this, we find the shortest path from the vehicle's current location to the current location's predecessor segment, then append the route from the current location to the destination station. In the case where the vehicle's current location has multiple predecessor segments (i.e. the current location is immediately downstream of a merge), the the shortest path is found to each of the predecessor segments and the path with the lowest total length is chosen. Track networks are required to be strongly connected, so a path from the current location to the predecessor must exist.

## Empty Vehicle Management ##

To decide where to send empty vehicles, the controller uses a calculation that is based on stations' demand for additional vehicles and on the distance from the vehicle to the stations.

A station's demand for additional vehicles is calculated as follows:

| **Condition** | **Demand Formula** |
|:--------------|:-------------------|
| All berths are full or reserved | `-Infinity`        |
| _pax_ > _vehicles_ | `pax - vehicles`   |
| _pax_ <= _vehicles_ | `(load_berths - vehicles) / (load_berths + 1.0)` |

where _pax_ is the number of passengers waiting at the station, _vehicles_ is the number of vehicles that are in the station or have reserved a berth in the station (reservations occur shortly before a vehicle enters the station), and _load\_berths_ is the total number of berths in the station capable of embarking passengers.

The first formula is intended to discourage empty vehicles from routing to a station that is already overloaded with vehicles. The second formula is the number of passengers in excess of the number of available vehicles. Each excess passenger provides one unit of station demand. The third formula is intended to route vehicles to stations which are low on vehicles when there are no excess passengers nearby. The formula is the fraction of unoccupied load berths, with the denominator slightly increased so as to keep the demand in the range [0,1).

When an empty vehicle is choosing a station to travel to, the station demands are multiplied by a distance factor _k_ where:

_k_` = max_dist / (9 * dist + max_dist)`

_max\_dist_ is the path distance from the vehicle to the furthest reachable station, and dist is the distance to the station under consideration. _k_ takes on a value of 1 when _dist_ is 0 and asymptotically approaches 0.1 as _dist_ approaches _max\_dist_. The magnitude of _k_ is relatively sensitive to small differences in distance when the distance is small, and relatively insensitive when distance is large.

Since station demand varies over time, empty vehicles reassess their destination periodically as they travel. Rather than recalculating on a fixed time or distance schedule, a vehicle will recalculate whenever it approaches a track split, using the principle that the best time to update information is when you are able to act upon it.

## Merging ##

The point at which two lines of tracks converge is referred to as a merge. There are three main types of merging:
  * Main line merges
  * Station merges outside a merge controller's Zone of Control
  * Station merges within a merge controller's Zone of Control

### _Main Line Merges_ ###

With a main line merge (aka non-station merge) the vehicles from two tracks are zippered together at speed rather than stopping one line of traffic or the other. A smooth zippering requires that appropriate gaps between vehicles are established in advance. The adjustments to relative vehicle positions required to create the gaps are orchestrated by a merge controller.

Each merge controller has a zone of control (ZoC). The ZoC starts at the merge point and extends upstream until it reaches the nearest non-station switch or merge point. The ZoC extends an equal length up both upstream legs, terminating at the shorter of the two legs. Switches and merges caused by stations are ignored when determining the ZoC length.

![http://img257.imageshack.us/img257/6120/mergezoneofcontrol.png](http://img257.imageshack.us/img257/6120/mergezoneofcontrol.png)

When a vehicle enters a merge controller's ZoC a _merge slot_ is created and added to the merge controller's _reservation queue_. The reservation queue is a FIFO queue containing all upcoming merge slots. Each merge slot contains a time span in which the vehicle will have exclusive access to the merge point. Vehicles are allocated merge slots on a first come, first served basis. The time span is calculated as:

```
start_time = max(current_time + ZoC_length / line_speed, reservation_queue[-1].end_time)
end_time = start_time + minimum_headway + vehicle_length / line_speed
```

where `reservation_queue[-1]` refers to the last element of the queue.

![http://img263.imageshack.us/img263/5476/mergequeue.png](http://img263.imageshack.us/img263/5476/mergequeue.png)

In addition to the time span, merge slots also contain a reference trajectory (in the form of a cubic spline) that will get the vehicle to the merge point at the allotted time. The spline is created using the [Trajectory Solver](TrajectorySolver.md)'s _target time_ function, which adjusts the cruising velocity downwards, if necessary, so that the vehicle arrives at the merge point at the correct time.

Vehicles which adjust their velocity do so only once they've fully entered the merge's zone of control and they return to line speed prior to passing through the merge point. Thus, any trajectory adjustments necessary for merging are limited to within the merge's ZoC and do not have an effect on vehicle flow outside of it.

Vehicles which are bound for a station within the ZoC still request a merge slot upon entering the ZoC. This is to ensure that they may be safely waived off from the station if the station is full or otherwise too congested. If the vehicle successfully enters its desired station within the ZoC, the merge slot is marked as 'relinquished' but is not removed from the merge's reservation queue.

TODO: Discuss headway shortening that occurs when two vehicles slow at the same point on the track, rather than slowing at the same time. Minimum velocity requirement, denying entrance to zone of control, and network topography limitations.

### _Station Merge Outside a ZoC_ ###

Outside of a merge controller's zone of control, vehicles travel at a predetermined velocity, referred to as the line speed. To exit a station, a vehicle accelerates to line speed on a separate line, referred to as the station's onramp, then merges into a gap in the vehicles on the main line.

Rather than actively creating a gap on the main line for the launching vehicle to merge into, this controller waits for a natural gap to occur. There are several reasons for this decision:
  * Simplicity. Vehicles on the main line are allowed to travel at a constant velocity when on the main line without need to slip forward or backwards.
  * Natural gaps are common. Due to the algorithm used, vehicles get tightly packed when travelling through a main line merge. This packing causes many small gaps to coalesce into fewer, larger gaps. Additionally, the station launch algorithms also create tightly packed groups of vehicles. In practice, it's rare for vehicles to end up with small gaps that could be feasibly exploited to create a larger gap.
  * Congestion management. If a line is so congested that naturally occurring gaps are rare or nonexistent, forcing additional vehicles onto the line encourages problems.

Detecting a gap to launch into is achieved by first calculating the length of time that the launching vehicle will require to reach the station's merge point. The distance that a vehicle on the main line would travel in that time is calculated, and the corresponding position on the upstream track is determined. The upstream point is expanded to account for vehicle length and desired headway to create a 'window'. If no vehicles are found in the window, then the launch vehicle may launch immediately and will have an unobstructed merge onto the main line. If one or more vehicles are found in the window, then the time required for them to clear the window is calculated, and the window is rechecked when that time has elapsed. Meanwhile, the launching vehicles stays stopped within the station. This continues until the launching vehicle finds the window to be clear, at which point it launches.

### _Station Merge Within a ZoC_ ###

When a station is located within a merge controller's zone of control, launching a vehicle from the station can no longer rely on simply checking the upstream track for conflicts. In this case, the vehicle also needs to ensure that there won't be a conflict at the main line merge further downstream, which it does by acquiring a merge slot prior to launching from the station.

Merge slots are typically created and granted on a first come-first served basis as vehicles enter the ZoC at the upstream boundary. As the slots are created, they are appended to the end of the merge's reservation queue. In the case where a vehicle is launching from a station located within a ZoC, one approach might call for a merge slot to be created as though the vehicle had just entered at the upstream boundary, so that the vehicle's merge slot is appended to the end of the reservation queue like normal. Like all merge slots, it would be accompanied by a vehicle trajectory, albeit one that it not being used. The launch of the vehicle from the station could be arranged so as to synchronize with the slot's trajectory at the station's merge, then utilizing that trajectory for the remainder of its journey to the merge point. The downside of this approach is that, while simple, every vehicle must wait for a phantom vehicle to progress from the edge of the ZoC to the station merge point before merging onto the main line. This delay reduces the rate at which vehicles can leave the station and its capacity.

An improved approach, and the one used by this controller, is to first look for a (time-based) gap in the merge's reservation queue into which a new merge slot could be inserted.  In order for a gap to be usable, it must have the following properties:
  * Reachable: The launching vehicle must not be required to travel faster than line speed in order to reach the merge point within the gap's time span.
  * Sufficient Width: The gap in the reservation queue must be at least `headway + (vehicle_length / line_speed)` seconds wide.
  * Unblocked: If the launch vehicle would need to overtake a vehicle on the same leg of the merge in order to reach the merge point within the gap's time span, then the gap is considered blocked and is unusable.

If a usable gap in the reservation queue is found, consider the two merge slots that bracket the gap, slot A and slot B. The trajectory associated with the slot A, the front-most slot of the pair, may be using a cruise velocity that is less than or equal to line speed. The trajectory associated with slot B, the rear-most slot, must have a cruise velocity that is equal to line speed, however. In order for the slot B trajectory to have a less than line speed cruise velocity, the merge slot reservation would need to have been delayed, but were that the case, there would not be a gap between the two slots -- they would be tightly packed instead.

_..._

## Station Behavior ##

Vehicle behavior within a station is governed by a state machine.

### _State Machine_ ###

![http://img253.imageshack.us/img253/6705/prtcontrolstatemachine.png](http://img253.imageshack.us/img253/6705/prtcontrolstatemachine.png)

Vehicles are in the RUNNING state when travelling between stations. Upon arriving at a station, if the vehicle has passengers on board then it will alternate between the UNLOAD ADVANCING and UNLOAD WAITING states as it makes its way to an unload berth. Otherwise it will go to a loading berth, alternating between LOAD ADVANCING and LOAD WAITING in the process. Assuming that it has passengers, once the vehicle is unable to advance any further, or once it has reached the front-most unloading berth, the vehicle will enter the DISEMBARKING state and begin to unload its passengers. Upon receiving the message stating that the passenger(s) have disembarked, the vehicle advances to the frontmost loading berth that it can reach. If there are passenger(s) available, the vehicle will enter the EMBARKING state and begin to load the passenger. Once loading has completed, the vehicle alternates between the EXIT WAITING and EXIT ADVANCING states until it reaches a launch berth. It enters the LAUNCH WAITING state and requests a launch time from the station (the details of which differ depending on the station's proximity to a merge). When the launch time arrives, the vehicle enters the LAUNCHING state and accelerates up to line speed and merges with the main line. Once the vehicle is fully on the main line, it enters the RUNNING state until it reaches its destination station and exits the main line again.

An attempt to transition out of an X\_WAITING state is triggered whenever another vehicle in the station moves (including exiting the station).