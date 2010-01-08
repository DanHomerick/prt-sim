"""Comm that uses a TCP socket.
"""
from __future__ import division # always use floating point division

import sys, socket, logging, heapq, struct, threading, time, Queue

import pyprt.shared.api_pb2 as api
import google.protobuf.text_format as text_format
import SimPy.SimulationRT as Sim

import globals
import layout

#import pdb  # python debugger

def receive(sock, queue, comm_idx):
    """A function that is responsible for receiving messages over a single,
    persistant TCP connection from an external process (e.g. a controller,
    or a GUI). Intended that this function run in a dedicated thread (it runs
    forever).

    socket: An already established TCP connection. May be the same socket that
        is also controlled by the transmit function.
    queue: a Queue.Queue, used to communicate the recieved messages to the
        globals.Interface.
    comm_idx: An index used to identify the comm in cases where the source of
        the message is important.

    Whenever a message is received from socket, a tuple is placed in the queue.
    The tuple is composed of:
        comm_idx, msg_type, msg_id, msg_time, body

    Raises an exception if msg_sep or msg_size is invalid.
    """
    while not globals.SimEnded:
        # Get msg_sep and msg_size
        hdr = []
        hdr_size = api.MSG_HEADER_SIZE
        while hdr_size:
            try:
                frag = sock.recv(hdr_size)
                hdr_size -= len(frag)
                hdr.append(frag)
            except socket.error:
                if globals.SimEnded:
                    return # let the thread die
                else:
                    raise
        hdr = ''.join(hdr) # concatenate strings

        msg_sep, msg_type, msgID, msg_time, msg_size \
                = struct.unpack('>hhiih', hdr)
        # TODO: Subtype Exception,  and handle
        if msg_sep != api.MSG_SEP:
            err_msg = api.SimMsgHdrInvalidSeparator()
            err_msg.msgID = msgID
            err_msg.msg_sep = msg_sep
            self.send(api.SIM_MSG_HDR_INVALID_SEPARATOR, err_msg)
            raise Exception, "Msg separator was incorrect: %d" % msg_sep

        if msg_size < 0:
            err_msg = api.SimMsgHdrInvalidSize()
            err_msg.msgID = msgID
            err_msg.msg_size = msg_size
            self.send(api.SIM_MSG_HDR_INVALID_SIZE, err_msg)
            raise Exception, "Negative msg_size received: %d" % msg_size

        body = []
        while msg_size:
            try:
                frag = sock.recv(msg_size)
                msg_size -= len(frag)
                body.append(frag)
            except socket.error: # socket has been shutdown
                if globals.SimEnded:
                    return # let the thread die
                else:
                    raise
        body = ''.join(body) # concatenate strings

        queue.put( (comm_idx, msg_type, msgID, msg_time, body) )
#        print "received and enqueued. msg_type:", msg_type, "msgID:", msgID

def transmit(sock, queue):
    """A function that is responsible for transmitting messages over a single,
    persistnant TCP connection to an external process (e.g. a controller,
    a logger, or a GUI). Intended that this function run in a dedicated thread.

    sock: An already established TCP connection. May be the same socket that
        is also controlled by the receive function.
    queue: A Queue.Queue, used to receive the message to be transmitted.

    The ControlInterface is responsible for closing down the socket.
    """
    while not globals.SimEnded:
        message = queue.get(block=True)
        sock.sendall(message)
        #queue.task_done()

# TODO: Refactor, based on the style used for the BaseController
class ControlInterface(Sim.Process):
    """TODO: Documentation.

    rt_comm: A boolean, where True indicates that the sim does not pause during
        communication with controllers.
    """
    def __init__(self, log):
        """log is either a filename, or an open file. If a filename, and the
        file already exists, it will be overwritten."""
        Sim.Process.__init__(self, name='CtrlInterface')
        if isinstance(log, str):
            if __debug__:
                self.log = open(log, 'w', buffering=1) # flush each line
            else:
                self.log = open(log, 'w')
        elif isinstance(log, file):
            self.log = log
        else:
            raise Exception("ControlInterface constructor expects a string or a file")

        self.TCP_server_socket = None
        self.UDP_server_socket = None
        self.TCP_client_sockets = list()
        self.UDP_client_addresses = list()
#        self.waitQ = list() # msgs (in form of tuples) to execute in the future
        self.receiveQ = Queue.Queue(-1) # no size limit
        self.sendQs = list() # of Queue.Queue's

        # list of booleans indicating whether each controller is ready to resume.
        # Only used during non-realtime operation.
        self.resume_list = list()

    def setup_server_sockets(self, TCP_port=None, UDP_port=None):
        """Creates sockets and binds them to the port numbers specified.

        TCP_port: The 'well known' TCP port number that clients connect to.
        UDP_port: The 'well known' UDP port number that clients send data to.

        Side-effects: Changes self.TCP_server_socket, self.UDP_server_socket
        """
        if TCP_port:
            self.TCP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Possible optimization: Have a 'Local machine only' flag in configuration,
            # and use 'localhost' instead of socket.gethostname(). The use of
            # the socket will bypass several network layers in that case (?)
            self.TCP_server_socket.bind( ('', TCP_port) ) # bind all available interfaces?
        if UDP_port:
            self.UDP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_server_socket.bind( ('', UDP_port) )
        self.TCP_server_socket.listen( min(128, socket.SOMAXCONN) ) # listen backlog

    def accept_connections(self, num_TCP_clients):
        """num_TCP_clients: How many TCP clients to expect.

        Blocks until the expected number of clients have connected.

        Allows any socket.error exceptions to propagate

        Side-effects: Changes self.TCP_client_sockets, self.TCP_client_handlers,
            self.receiveQ, self.sendQs
        """

        for idx in xrange(num_TCP_clients):
            sock, address = self.TCP_server_socket.accept()
            print "Established TCP connection with", sock.getsockname()
            self.TCP_client_sockets.append(sock)
            self.resume_list.append(False) # wait for a resume before commencing
            receive_thread = threading.Thread(target = receive,
                                              name='sim_listener_'+str(idx),
                                              args = (sock, self.receiveQ, idx))
            receive_thread.setDaemon(True)
            receive_thread.start()

            sendQ = Queue.Queue(-1) # no size limit
            self.sendQs.append(sendQ)
            transmit_thread = threading.Thread(target = transmit,
                                           name='sim_transmitter_'+str(idx),
                                           args = (sock, sendQ))
            transmit_thread.setDaemon(True)
            transmit_thread.start()

        # close the 'well known' port, since no more TCP connections are expected
        self.TCP_server_socket.close()
        self.TCP_server_socket = None

    def disconnect(self):
        if self.TCP_server_socket:
            self.TCP_server_socket.close()

        # wait until all send queues are empty
        while [True for q in self.sendQs if not q.empty()]:
            time.sleep(0.001)
            print "waiting on sendQs"

        for sock in self.TCP_client_sockets:
            try:
                sock.shutdown(socket.SHUT_RDWR)
                sock.close()
            except:
                pass

        self.TCP_client_sockets = list()

    def talk(self):
        """Main loop"""
        #### NON-realtime version ####
        while True:
            # Until all controllers send a CtrlResume, wait for new imput and handle it
            # Set up in the python equivalent to a do-while loop to tolerate
            # the behaviour of the Alarm Clock class.
            while True:
                self.handle_msg()
                if all(self.resume_list):
                    break
                else:
                    logging.debug("Waiting for other controllers to resume: %s", self.resume_list)

            # Once all controllers have sent a CtrlResume, passivate self. Will
            # be reactivated once any part of the sim sends out a message.
            yield Sim.passivate, self # allow rest of sim to run


    def handle_msg(self):
        """Expects a numeric msg_type and a msg_str in the format specified by api.proto
        """
        comm_idx, msg_type, msgID, msg_time, msg_str = self.receiveQ.get(block=True)
#        print "pulled from receive queue. msg_type:", msg_type, "msgID:", msgID

        if msg_time == ms_now():
            try:
                # COMMANDS
                if msg_type == api.CTRL_RESUME:
                    # No response, just resume Simulation
                    msg = api.CtrlResume()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    # Ignore Resume if ctrl hasn't seen latest msg from Sim
                    if msg.last_sim_msgID == globals.msg_id_widget.last_id():
                        # set the 'resume' that corresponds to the controller to True
                        self.resume_list[comm_idx] = True
                        logging.debug("Processed current resume. msgID:%s, msg_time:%s", msgID, msg_time )
                    else:
                        logging.debug("Processed stale resume. msgID:%s, msg_time:%s", msgID, msg_time )

                elif msg_type == api.CTRL_CMD_VEHICLE_TRAJECTORY:
                    msg = api.CtrlCmdVehicleTrajectory()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    v.process_spline(msg.spline )
#                    ma, md, mj = self.validate_speed_params(v, msg) # also converts from cm to m
#                    v.set_speed(msg.target_speed/100, ma, md, mj, msgID) # cm/s -> m/s

                elif msg_type == api.CTRL_CMD_SWITCH:
                    msg = api.CtrlCmdSwitch()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    sw = self.get_switch(msg.swID)
                    ts = self.get_trackSeg(msg.eID)
                    # make a list of valid outgoing edges
                    out_edges = [data for src, sink, data in globals.DiGraph.edges(sw, data=True)]
                    if ts not in out_edges:
                        raise globals.InvalidTrackSegID, msg.eID
                    v = self.get_vehicle(msg.vID)
                    sw.routing_table[v.ID] = (ts, msgID)

#                elif msg_type == api.CTRL_CMD_STATION_LAUNCH:
#                    msg = api.CtrlCmdStationLaunch()
#                    msg.ParseFromString(msg_str)
#                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
#                    s = self.get_station(msg.sID)
#                    v = self.get_vehicle(msg.vID)
#                    if v.loc is not s:
#                        raise globals.InvalidStationID, msg.sID
#                    ma, md, mj = self.validate_speed_params(v, msg)
#                    s.launch(v, msg.target_speed/100, ma, md, mj, msgID) # cm/s -> m/s

#                elif msg_type == api.CTRL_CMD_PASSENGER_LOAD_VEHICLE:
#                    msg = api.CtrlCmdPassengerLoadVehicle()
#                    msg.ParseFromString(msg_str)
#                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
#                    stat = self.get_station(msg.sID)
#                    v = self.get_vehicle(msg.vID)
#                    pax = self.get_passenger(msg.pID)
#                    stat.board_passenger(v, pax, msgID)

                # REQUESTS
                elif msg_type == api.CTRL_REQUEST_VEHICLE_STATUS:
                    msg = api.CtrlRequestVehicleStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    resp = api.SimResponseVehicleStatus()
                    resp.msgID = msgID
                    v.fill_VehicleStatus(resp.v_status)
                    self.send(api.SIM_RESPONSE_VEHICLE_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_STATION_STATUS:
                    msg = api.CtrlRequestStationStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    stat = self.get_station(msg.sID)
                    resp = api.SimResponseStationStatus()
                    resp.msgID = msgID
                    stat.fill_StationStatus(resp.s_status)
                    self.send(api.SIM_RESPONSE_STATION_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_STATION_SUMMARY:
                    msg = api.CtrlRequestStationSummary()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    stat = self.get_station(msg.sID)
                    resp = api.SimResponseStationSummary()
                    resp.msgID = msgID
                    stat.fill_StationSummary(resp.s_summary)
                    self.send(api.SIM_RESPONSE_STATION_SUMMARY, resp)

                elif msg_type == api.CTRL_REQUEST_PASSENGER_STATUS:
                    msg = api.CtrlRequestPassengerStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    p = self.get_passenger(msg.pID)
                    resp = api.SimResponsePassengerStatus()
                    resp.msgID = msgID
                    p.fill_PassengerStatus(resp.p_status)
                    self.send(api.SIM_RESPONSE_PASSENGER_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_SWITCH_STATUS:
                    msg = api.CtrlRequestSwitchStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    sw = self.get_switch(msg.swID)
                    resp = api.SimResponseSwitchStatus()
                    resp.msgID = msgID
                    sw.fill_SwitchStatus(resp.sw_status)
                    self.send(api.SIM_RESPONSE_SWITCH_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_TRACKSEGMENT_STATUS:
                    msg = api.CtrlRequestTrackSegmentStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    ts = self.get_trackSeg(msg.eID)
                    resp = api.SimResponseTrackSegmentStatus()
                    resp.msgID = msgID
                    ts.fill_TrackSegmentStatus(resp.ts_status)
                    self.send(api.SIM_RESPONSE_TRACKSEGMENT_STATUS, resp)


                # NOTIFICATIONS

                # processing is delayed until correct time reached
                elif msg_type == api.CTRL_SETNOTIFY_TIME:
                    msg = api.CtrlSetnotifyTime()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    resp = api.SimNotifyTime()
                    resp.msgID = msgID
                    resp.time = ms_now() # converted from seconds to milliseconds
                    self.send(api.SIM_NOTIFY_TIME, resp)

                # Works, but spams the log with erroneous RCVD messages
                # TODO: Move to a vehicle co-routine, which waits on
                # end_segment and end_duration signals.
                elif msg_type == api.CTRL_SETNOTIFY_VEHICLE_POSITION:
                    msg = api.CtrlSetnotifyVehiclePosition()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    loc = self.get_trackSeg(msg.eID)
                    if v.loc is loc and globals.dist_eql(v.pos, msg.pos/100): # convert from cm to meters
                        resp = api.SimNotifyVehiclePosition()
                        resp.msgID = msgID
                        resp.vID = self.ID
                        resp.eID = self.loc.ID
                        resp.pos = int(self.pos*100) # meters to cm
                        self.send(api.SIM_NOTIFY_VEHICLE_POSITION, resp)

                    # Target position ahead of vehicle on current edge
                    if v.loc is loc and msg.pos/100 > v.pos:
                        time = v.calc_time_to_pos(msg.pos/100)
                    # Target position on another edge, or behind current pos
                    else:
                        time = v.calc_time_to_pos(v.loc.length)
                    print "Time to notification pos, or end of seg:", time
                    if not time: # pos eqn expires first
                        time = v.pos_eqn_duration
                    AlarmClock(time)
                    time_ms = int(round(time,3)*1000) # ??? Remove?
                    self.waitQ.append( (msg_type, msgID, msg_time, msg_str) )

                else:
                    resp = api.SimUnimplemented()
                    self.send(api.SIM_UNIMPLEMENTED, resp)
                    logging.debug("Msg type %d received, but not yet implemented.",
                                  msg_type)

            except globals.InvalidMsgType:
                err_msg = api.SimMsgHdrInvalidType()
                err_msg.msgID = msgID
                err_msg.msg_type = msg_type
                self.send(api.SIM_MSG_HDR_INVALID_TYPE, err_msg)
                logging.error("Received invalid msg type: %s", msg_type)
                globals.errors += 1
            except globals.InvalidTrackSegID, e:
                err_msg = api.SimMsgBodyInvalidID()
                err_msg.msgID = msgID
                err_msg.loc_type = api.TRACKSEGMENT
                err_msg.locID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except globals.InvalidWaypointID, e:
                err_msg = api.SimMsgBodyInvalidID()
                err_msg.msgID = msgID
                err_msg.loc_type = api.WAYPOINT
                err_msg.locID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except globals.InvalidSwitchID, e:
                err_msg = api.SimMsgBodyInvalidID()
                err_msg.msgID = msgID
                err_msg.loc_type = api.SWITCH
                err_msg.locID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except globals.InvalidMergeID, e:
                err_msg = api.SimMsgBodyInvalidID()
                err_msg.msgID = msgID
                err_msg.loc_type = api.MERGE
                err_msg.locID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except globals.InvalidStationID, e:
                err_msg = api.SimMsgBodyInvalidID()
                err_msg.msgID = msgID
                err_msg.loc_type = api.STATION
                err_msg.locID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except globals.InvalidVehicleID, e:
                err_msg = api.SimMsgBodyInvalidID()
                err_msg.msgID = msgID
                err_msg.loc_type = api.VEHICLE
                err_msg.locID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except globals.InvalidPassengerID, e:
                err_msg = api.SimMsgBodyInvalidID()
                err_msg.msgID = msgID
                err_msg.loc_type = api.PASSENGER
                err_msg.locID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)

        # if packet time is in future, set an alarm to wake up interface
        # at the time, and revive the message then.
        elif msg_time > ms_now():
            AlarmClock(msg_time / 1000.0, (comm_idx, msg_type, msgID, msg_time, msg_str) ) # convert from ms to sec
        else: # msg_time < ms_now()
            err = api.SimMsgHdrInvalidTime()
            err.msgID = msgID
            err.msg_time = msg_time
            self.send(api.SIM_MSG_HDR_INVALID_TIME, err)
            logging.error("Time stamp for RCVD message %s type %s in the past. Stamp: %s Now: %s ",
                  msgID, msg_type, msg_time, ms_now())
            globals.errors += 1

        return msg_type

    def send(self, msg_type, msg):
        """Send an outgoing message.
        Every message sent is prefixed with:
               msg seperator: 2 bytes, signed -32123
               msg type: 2 bytes, signed
               msg ID: 4 bytes, signed
               msg time: 4 bytes, signed
               msg size: 2 bytes, signed (max msg size of 32768 bytes)
        The message seperator is always -32123
        The msg type is one of the values found in CtrlMsgType or SimMsgType
        The msg size is the length (in bytes) of the serialized message.
        These values are transmitted in network byte order (Big Endian).

        TEMP: Transmits to all clients (no filtering)
        """
        msgID = globals.msg_id_widget.next_id()
        msg_time = ms_now()
        msg_str = msg.SerializeToString()
        msg_size = len(msg_str)
        # big_endian, *std size* and align: short, short, int, int, short
        # short = 2 bytes, int = 4 bytes
        header = struct.pack('>hhiih', api.MSG_SEP, msg_type, msgID,
                             msg_time, msg_size)
        message = header + msg_str

        # TODO: Only send to consumers that are subscribed to that message type
        # and/or message target.
        for q in self.sendQs:
            q.put(message)

        if self.passive() and not globals.SimEnded:
            # Reset the resume_list to all False
            self.resume_list[:] = [False] * len(self.resume_list)
            # Reactivate the ControlInterface, placing it at the end of the
            # list of events to run.
            Sim.reactivate(self, prior=False)

        print >> self.log, "Now:%s, SENT, %s, MsgType:%d, MsgID:%d, MsgTime:%d" %\
                 (time.time(), msg.DESCRIPTOR.name, msg_type, msgID, msg_time)
        text_format.PrintMessage(msg, out=self.log)
        print >> self.log, "--"

    def log_rcvd_msg(self, msg_type, msgID, msg_time, msg):
        # log msg
        print >> self.log, "Now:%s, RCVD, %s, MsgType:%d, MsgID:%d, MsgTime:%d" %\
                 (time.time(), msg.DESCRIPTOR.name, msg_type, msgID, msg_time)
        text_format.PrintMessage(msg, out=self.log)
        print >> self.log, "--"
        if __debug__:
            self.log.flush()


    def get_trackSeg(self, ID):
        """"Return the TrackSegment object specified by the ID."""
        try:
            ts = globals.TrackSegments[ID]
        except KeyError:
            raise globals.InvalidTrackSegID, ID
        return ts

    def get_switch(self, ID):
        """"Return the switch object specified by the ID."""
        try:
            sw = globals.Switches[ID]
        except KeyError:
            raise globals.InvalidSwitchID, ID
        if not isinstance(sw, layout.Switch):
            raise globals.InvalidSwitchID, ID
        return sw

    def get_station(self, ID):
        """"Return the station object specified by the ID."""
        try:
            s = globals.Stations[ID]
        except KeyError:
            raise globals.InvalidStationID, ID
        if not isinstance(s, layout.Station):
            raise globals.InvalidStationID, ID
        return s

    def get_vehicle(self, ID):
        """Return the vehicle object specified by the ID."""
        try:
            v = globals.Vehicles[ID]
        except KeyError:
            raise globals.InvalidVehicleID, ID
        return v

    def get_passenger(self, ID):
        """"Return the passenger object specified by the ID."""
        try:
            pax = globals.Passengers[ID]
        except KeyError:
            raise globals.InvalidPassengerID, ID
        return pax

    # DEPRECATED
    def validate_speed_params(self, v, msg):
        """ Validate/set max_accel, max_jerk on a per vehicle
         basis to accomodate variety of vehicle capabilities
         or passenger limitations.

         Assumed that the controller may not have a Floating Point Unit, so
         all communications are use centimeters rather than meters as the base
         unit of length. This function converts back to meters in addition
         to validating."""
        import warnings
        warnings.warn('deprecated function')
        # regular case
        if not msg.emergency:
            # if max_accel specified by controller, validate it
            if msg.max_accel:
                max_accel = msg.max_accel/100 # cm/s^2 -> m/s^2
                if max_accel > v.accel_max_norm or max_accel < 0:
                    raise globals.InvalidAccel, max_accel
                ma = max_accel
            # otherwise set it
            else: ma = v.accel_max_norm

            if msg.max_decel:
                max_decel = msg.max_decel/100  # cm/s^2 -> m/s^2
                if max_decel < v.accel_min_norm or max_decel > 0:
                    raise globals.InvalidDecel, max_decel
                md = max_decel
            else: md = v.accel_min_norm

            if msg.max_jerk:
                max_jerk = msg.max_jerk/100 # cm/s^2 -> m/s^2
                mj = abs(max_jerk)
                if mj > v.jerk_max_norm:
                    raise globals.InvalidJerk, max_jerk
            else: mj = abs(v.jerk_max_norm)
        # emergency case
        else:
            # if max_accel specified by controller, validate it
            if msg.max_accel:
                max_accel = msg.max_accel/100 # cm/s^2 -> m/s^2
                if max_accel > v.accel_max_emerg or max_accel < 0:
                    raise globals.InvalidAccel, max_accel
                ma = max_accel
            # otherwise set it
            else: ma = v.accel_max_emerg

            if msg.max_decel:
                max_decel = msg.max_decel / 100 # cm/s^2 -> m/s^2
                if max_decel < v.accel_min_emerg or max_decel > 0:
                    raise globals.InvalidDecel, max_decel
                md = max_decel
            else: md = v.accel_min_emerg

            if msg.max_jerk:
                max_jerk = msg.max_jerk / 100 # cm/s^3 -> m/s^3
                mj = abs(max_jerk)
                if mj > v.jerk_max_emerg:
                    raise globals.InvalidJerk, max_jerk
            else: mj = v.jerk_max_emerg
        return (ma, md, mj)

    def fill_TotalStatus(self, ts, msgID=None):
        """Fills a api.TotalStatus message with current data.

        ts: an empty api.TotalStatus message
        msgID: The msgID that this is a response to, if any."""
        if msgID:
            ts.msgID = msgID
        for v in globals.vehicle_list:
            v_status = api.VehicleStatus()
            v.fill_VehicleStatus(v_status)
            ts.v_statuses.append(v_status)
        # send track segment status for all track segments
        for ts in globals.TrackSegments.itervalues():
            ts_status = api.TrackSegmentStatus()
            ts.fill_TrackSegmentStatus(ts_status)
            ts.ts_statuses.append(ts_status)
        # send Switch status msgs
        for switch in globals.switch_list:
            sw_status = api.SwitchStatus()
            switch.fill_SwitchStatus(sw_status)
            ts.sw_statuses.append(sw_status)
        # send Station status msgs and summary msgs
        for station in globals.station_list:
            s_status = api.StationStatus()
            station.fill_StationStatus(s_status)
            ts.s_statuses.append(s_status)

class AlarmClock(Sim.Process):
    """A Sim.Process that wakes up at time and places a message in the receive
    message queue. Used to delay processing of a message.

    time: The (simulation) time that the alarm should go off, depositing it's
        payload, if any.
    receive: The payload. A tuple containing the fields expected by the
        ControlInterface.receiveQ queue.
"""
    def __init__(self, time, receive=None):
        Sim.Process.__init__(self, name="AlarmClock")
        self.time = time
        self.receive = receive
        Sim.activate(self, self.ring())
    def ring(self):
        yield Sim.hold, self, self.time - Sim.now()
        if self.receive:
            globals.Interface.receiveQ.put(self.receive)
        if globals.Interface.passive():
            Sim.reactivate(globals.Interface, prior=False)

def ms_now():
    """Return Sim.now() in integer form, rounded to milliseconds. This is the
    form that is used for communication with the Control modules."""
    return int(round(Sim.now()*1000))
