## Project Goals ##

Provide a modular, cross-platform, open-source simulator for Personal Rapid Transit systems.

## What is Personal Rapid Transit ##

Personal Rapid Transit (PRT) is a type of mass transit system that uses small, autonomous vehicles. Whereas conventional mass transit uses large vehicles that run on a fixed schedule and travel a fixed route, PRT operates more like a fleet of taxi cabs in that each vehicle runs on-demand and travels directly to the passenger's destination station. One of PRT's principle goals is to provide faster and more convenient travel than could be achieved with traditional mass transit.

## Project ##

There are four main components of the project:
  1. **[TrackBuilder](TrackBuilder.md):** For laying out a PRT system's track and station network. Integrates Google Maps data.
  1. **[Simulator](Simulator.md):** The core simulator. Designed to be flexible enough to simulate most PRT systems.
  1. **[Simulation GUI](SimViz.md):** A simple 2D visualization that provides a 'live' view of the simulation as it progresses, as well as a user interface for controlling the simulation.
  1. **[Vehicle Control](Controllers.md):** The vehicle's control is not 'baked in' to the simulator. Control is handled by one or more external processes that communicate with the Simulator via an API.

## Installation ##

There are two separate installers for the project at this time.

One of the installers is for TrackBuilder. This installer is an '.air' file, and requires that [Adobe AIR](http://get.adobe.com/air/) be installed in order to run. TrackBuilder can run on Windows, Macintosh, or Linux. The remaining three components only have an installer for Windows at this time. Macintosh and Linux will be supported in the future.

Both installers are available in the [downloads](http://code.google.com/p/prt-sim/downloads/list) section. See the [Getting Started](GettingStarted.md) section for more information.