## Introduction ##
A word of warning -- this project is still under development, and is frankly not quite ready for use by end users. It is getting very close to ready, but there are still numerous bugs, especially when using the installer rather than a SVN checkout.

## Installation ##

There are two separate installers for the project at this time.

One of the installers is for TrackBuilder. This installer is an '.air' file, and requires that [Adobe AIR](http://get.adobe.com/air/) be installed in order to run. TrackBuilder can run on Windows, Macintosh, or Linux. The remaining three components only have an installer for Windows at this time. Macintosh and Linux will be supported in the future.

Both installers are available in the [downloads](http://code.google.com/p/prt-sim/downloads/list) section.

For instructions for using a SVN checkout see this [page](GettingSimRunning.md). Not recommended for regular users.

## Running a Scenario ##

Each scenario has a "config.ini" file that acts as a project file. The simplest way to start a simulation is by opening one of these config files from within the Simulator.

Step by step directions:
  * Launch the 'PRT Simulator'
  * File -> Open Configuration File ...
    * Browse to the desired scenario folder, and open the desired config file. Most of the existing scenarios are for development purposes and are very limited. Some are still buggy. The Santa Cruz scenario is the largest scenario included at this time.
  * Simulation -> Connect External Controller ...
  * Simulation -> Start Sim

The simulation speed can be controlled from the Simulation menu. Once the simulation is completed, a report summarizing the run is generated and displayed. The report generation may take several minutes on a slower machine.

## Creating a Simple Scenario ##
Creating scenarios is mostly done through TrackBuilder, although some additional files still need to be created by hand.

TODO: Write tutorial for creating a loop with two stations and several vehicles.