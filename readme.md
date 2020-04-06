# Warning - Dead Project
This was my master's project for my Comp Sci degree, but hasn't been touched in more than a decade. It has died from bit rot, and would require a significant effort to resurrect. This repo is an export from SVN, with most of the commit history lost.

# Summary
This project is a simulator for Personal Rapid Transit (PRT) networks. PRT takes the idea that you can "easily" have autonomous vehicles if you separate them from everything else. That is, give them their own "road" network, such as an elevated trackway or an underground tunnel. This sim was written to be generic, but was done in collaboration with a PRT startup ([SkyTran](https://www.skytran.com/)).

The main components are:
* [TrackBuilder](track_builder) - a GUI for creating your network
* [Simulator](pyprt/sim) - Simulation and visualization for the multi-vehicle transit network
* [Controller](pyprt/ctrl) - Vehicle controller
