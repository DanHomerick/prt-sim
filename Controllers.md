## Overview ##

The simulator has been designed from the start to cleanly divorce the vehicle control from the simulation. A vehicle controller is a standalone program that communicates with the simulator via a network port. This allows the vehicle control logic to be implemented in (almost) any language, and to run on any hardware with adequate network support.

## Included Controllers ##

The project includes two controllers, though writing your own is encouraged! The [PRT Controller](PRT_Controller.md) implements a PRT service mode, where vehicles are routed on-demand and there is no fixed schedule. The [GTF Controller](GTF_Controller.md) is designed to simulate conventional mass transit, in which relatively high capacity vehicles travel fixed routes on a fixed schedule. GTF stands for [Google Transit Feed](http://code.google.com/transit/spec/transit_feed_specification.html), which is a data format for publishing scehdule and route information, which is indirectly used by this controller.