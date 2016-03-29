# Overview #
TrackBuilder is a graphical tool for laying out a guideway for a PRT network.

Features:
  * Google Maps data
  * Guideway laid out with combination of straight segments and constant radius curves
  * Collects elevation data
  * Live preview of track placement
  * Infinite Undo
  * Imports Google Transit Feeds for simulating existing mass transit networks


### Google Maps ###
Rather than forcing users to import and register their own maps data, provides Google Maps data as a background.

### Constant Radius Curves ###
Use of constant radius curves for turns helps to create more realistic network layouts. TrackBuilder provides quick feedback about how much lateral acceleration will be felt by passengers at a given curve radius and speed (assuming no superelevation).

### Elevation Data ###
While not yet used by the simulation, elevation data for the track is collected.

### Google Transit Feeds ###
PRT is often envisioned as a feeder network for a train station or as a complement to a bus system. Many transit agencies supply their route and schedule information to Google in  Google Transit Feed (GTF) [format](http://code.google.com/transit/spec/transit_feed_specification.html). While most of these feeds are private, some agencies have chosen to make their feeds [publicly](http://code.google.com/p/googletransitdatafeed/wiki/PublicFeeds) available.

TrackBuilder can import a feed and provides a wizard for choosing which schedules and routes to import. Extra data from the feed is appended to the regular layout data and allows the gtf\_controller to simulate the desired routes and schedules.

Even in cases where a public feed is not available, the GTF format is simple enough that recreating a few routes and schedules by hand can be practical.

Caveats: The "tracks" created by the GTF import can be pretty hideous. Also, while the gtf import has been tested and used for bus data, other types (such as rail) haven't been tested. It _should_ work, but...

### General Usage Tips ###
  * Hit the ESC key to go back to moving the origin marker (blue circle).
  * Switching to Satellite view may allow you to zoom in further than Map view.
  * When layout is saved, a snapshot of the current map view is saved and is used by the Sim as a background image. For best results, you will need to manually zoom and center the map prior to your last Save.
  * Run Tools->Validate and Save often.

## About the Name ##
Have a better name for this thing than "TrackBuilder"? Let me know!