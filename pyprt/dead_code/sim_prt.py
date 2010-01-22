#!/usr/bin/env python
# '/' is true division, '//' is truncating division
from __future__ import division 

import logging
import sys
import string
from optparse import OptionParser

#import matplotlib          # temp
#matplotlib.use('WXAgg')    # temp
#import matplotlib.pylab as P
import google.protobuf.text_format as text_format

import globals

def sort_by_vehicle_pos(x, y):
    # if x and y are both Track pieces (i.e. have a position) and are the same
    # location
    if isinstance(x.loc, layout.Track) and isinstance(y.loc, layout.Track) \
       and x.loc is y.loc:
        return cmp(y.pos, x.pos) # sort by position, descending order
    else:
        return cmp(x.loc, y.loc) # else sort by place

def start_sim(options=None):
    """Activate the SimPy Processes and start the simulation."""
    # initialize SimPy 
    Sim.initialize()
    
    # Creating a vehicle that is already on a track segment (TrackSegment) is a bit odd,
    # and is only done during startup. I reorder the vehicles so that the queue
    # will have proper FIFO ordering on the TrackSegment.
    vehicle_list = globals.vehicles.values()  # the Vehicle instances
    vehicle_list.sort(cmp=sort_by_vehicle_pos)
    
    # activate the vehicles
    for v in vehicle_list:
        Sim.activate(v, v.ctrl_loop()) 

    # activate the stations
    for s in globals.station_list:
        Sim.activate(s, s.ctrl_loop())

    # activate the event manager
    Sim.activate(globals.event_manager, globals.event_manager.spawn_events())
    
    # activate the visualization controller's data collection
    if options and options.render:
        Sim.activate(globals.Viz, globals.Viz.collect_data())

    # Establish communication with control module and activate the interface.
    Sim.activate(globals.interface, globals.interface.talk())

    # start the sim
    end_time = config.conf.getfloat('Simulation', 'sim_end_time')
    if globals.real_time:
        Sim.simulate(until=end_time, real_time=globals.real_time, rel_speed=1)
    else:
        Sim.simulate(until=end_time)

def print_postsim_status():
    end_time = config.conf.getfloat('Simulation', 'sim_end_time')
    print "\nPost Sim Vehicle Status"
    vehicle_list = globals.vehicles.values()
    print "%4s%15s%15s%15s%15s" \
        % ('vID', 'vPos', 'vSpeed', 'location', 'distTrav')
    for v in vehicle_list:
        print "%4d%15s%15s%15s%15d" \
                % (v.ID, v.pos, v.speed, v.loc, v.path.get_dist_travelled())
    
    print "\n Post Sim Vehicle Crash Report (Multiple crashes may be reported for the same incident)"
    print ("%4s"+"%12s"+"%15s%15s"+"%15s"+"%10s%10s%10s") \
           % ('vID', 'time', 'loc', 'pos', 'leadVehicle', 'rearend',
              'sideswipe', 'occurred')
    for v in vehicle_list:
        for crash in v.collisions:        
            print ("%4d"+"%12.3f"+"%15s%15s"+"%15d"+"%10s%10s%10s") \
                % (v.ID, crash.time, crash.trav.loc,
                   crash.trav.get_pos_from_time(crash.time),
                   crash.lv.ID, crash.rearend, crash.sideswipe, crash.occurred)
                    
    print "\nPost Sim Passenger Report"
    print ("%4s" + "%15s"*6) \
            % ('pID','srcStat','destStat','curLoc','waitT','TravelT','Success')
    for p in globals.passengers.itervalues():
        if p.trip_end:
            travel = p.trip_end - p.trip_boarded
            wait = p.trip_boarded - p.trip_start
        elif p.trip_boarded:
            travel = end_time - p.trip_boarded
            wait = p.trip_boarded - p.trip_start
        else:
            travel = 0.0
            wait = end_time - p.trip_start
        print ("%4d" + "%15s"*3 + "%15.3f"*2 + "%15s") \
                % (p.ID, p.src_station, p.dest_station,
                   p.loc, wait, travel, p.trip_success)

    print "\nPost Sim Switch Report"
    print "%4s%15s%10s" % ('ID', 'Name', 'Errors')
    for sw in globals.switch_list:
        print "%4d%15s%10d" % (sw.ID, sw, sw.errors)
                            
    print "\nPost Sim Station Report"
    print ("%4s%17s" + "%10s"*7) % ('ID', 'Name', 'Departs', 'Arrives', 'Crashes', 'Unload', 'Queue', 'Load', 'Storage')
# unload (slots), empty_queue (slots), load (slots), storage (slots)
    for s in globals.station_list:
        print ("%4d%17s" + "%10d"*7) % \
                (s.ID, s.label, s.totalArrivals, s.totalDepartures, s.totalCrashes, 
                 s.num_load_berths, s.num_queue_slots, s.num_load_berths,
                 s.num_storage_slots)
                
def main():
    # load all configuration info from config file
    if not args:
        print optpar.error("Must supply a config-file. Use ../doc/sampleConf.txt as a sample file.")
    config.initialize(args[0], options)   

    if options and options.port is not None:
        TCP_port = options.port
    else:
        TCP_port = config.conf.getint('Simulation', 'TCP_port')        

    globals.interface.connect(TCP_port)

    if options.profile:
        import profile
        print "Profiling ..."
        profile.run('start_sim', options.profile)
        end = api.SimEnd()
        globals.interface.quick_send(api.SIM_END, end)
        print "Disconnecting"
        globals.interface.disconnect()       
        
    else:
        try:    
            start_sim(options)
            end = api.SimEnd()
            globals.interface.quick_send(api.SIM_END, end)
        finally:
            print "Disconnecting"
            globals.interface.disconnect()


#    P.savefig('track.png', format='PNG')
    print_postsim_status()
    if options.plot:
        veh = globals.vehicles.values()
        if len(veh) == 1:
            veh[0].path.plot(str(veh[0]), show=True, new_fig=False)
        else:
            veh[0].path.plot(str(veh[0]), show=False, new_fig=False)
            for v in veh[1:-1]:
                v.path.plot(str(v), show=False, new_fig=True)
            veh[-1].path.plot(str(veh[-1]), show=True, new_fig=True)
            
    if options.render:
       globals.Viz.run_animation()    

if __name__ == '__main__':
    # Parse the command line arguments
    optpar = OptionParser(usage="usage: %prog [options] config-file")
    optpar.add_option("-v", "--verbose", action="store_true",
                      help="More spam, please!")
    optpar.add_option("-r", "--render", action="store_true", dest="render",
                      default=False, help="Render animation")
    optpar.add_option("--plot", action="store_true", dest="plot",
                      help="Plot the path for each vehicle")
    optpar.add_option("-l", "--logfile", dest="logfile", metavar="FILE",
                      help="Log information to FILE. Overrides logfile specified \
                      in configuration. Will accept 'stdout'.")
    optpar.add_option("--profile", dest="profile", metavar="FILE",
                      help="Profile this run, storing the results in FILE.")
    optpar.add_option("-p", "--port", action="store", type="int",
                      help="Specify what port to listen on.")
    optpar.add_option("-t", "--trace", action="store_true", default=False,
                      help="Print Sim Trace messages.")
    options, args = optpar.parse_args()
    
    globals.trace = options.trace
    
    if options.trace:
        import SimPy.SimulationTrace as Sim
    else:
        import SimPy.Simulation as Sim

    # Import other prt modules after the flavor of SimPy has been decided on
    import pyprt.shared.api_pb2 as api
    import config
    import vehicle
    import layout

    main()
    