"""Comm that uses a TCP socket.
TODO: Rename pyprt.ctrl.base_controller to pyprt.shared.comm and inherit from that.
"""
from __future__ import division # always use floating point division

import sys, socket, logging, heapq, struct, threading, time, Queue, os

import SimPy.SimulationRT as Sim
import google.protobuf.text_format as text_format

from pyprt.shared.utility import pairwise
import pyprt.shared.utility as utility
import pyprt.shared.api_pb2 as api
import common
import main

#import pdb  # python debugger

def receive(sock, queue, comm_idx):
    """A function that is responsible for receiving messages over a single,
    persistant TCP connection from an external process (e.g. a controller,
    or a GUI). Intended that this function run in a dedicated thread (it runs
    forever).

    socket: An already established TCP connection. May be the same socket that
        is also controlled by the transmit function.
    queue: a Queue.Queue, used to communicate the recieved messages to the
        common.interface.
    comm_idx: An index used to identify the comm in cases where the source of
        the message is important.

    Whenever a message is received from socket, a tuple is placed in the queue.
    The tuple is composed of:
        comm_idx, msg_type, msg_id, msg_time, body

    Raises an exception if msg_sep or msg_size is invalid.
    """
    while not common.sim_ended:
        # Get msg_sep and msg_size
        hdr = []
        hdr_size = api.MSG_HEADER_SIZE
        while hdr_size:
            try:
                frag = sock.recv(hdr_size)
                hdr_size -= len(frag)
                hdr.append(frag)
            except socket.error:
                if common.sim_ended:
                    return # let the thread die
                else:
                    raise
        hdr = ''.join(hdr) # concatenate strings

        msg_sep, msg_type, msgID, msg_time, msg_size \
               = struct.unpack('>hhiii', hdr)
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
                if common.sim_ended:
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
    while not common.sim_ended:
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
            self.log = os.devnull

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
            self.TCP_server_socket.bind( ('localhost', TCP_port) ) # bind all available interfaces?
        if UDP_port:
            self.UDP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_server_socket.bind( ('localhost', UDP_port) )
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

        greeting = self.make_SimGreeting()
        self.send(api.SIM_GREETING, greeting)

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
                    pass
##                    logging.debug("Waiting for other controllers to resume: %s", self.resume_list)

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
                    if msg.last_sim_msgID == common.msg_id_widget.last_id():
                        # set the 'resume' that corresponds to the controller to True
                        self.resume_list[comm_idx] = True
##                        logging.debug("Processed current resume. msgID:%s, msg_time:%s", msgID, msg_time )
                    else:
                        pass
##                        logging.debug("Processed stale resume. msgID:%s, msg_time:%s", msgID, msg_time )

                elif msg_type == api.CTRL_CMD_VEHICLE_TRAJECTORY:
                    msg = api.CtrlCmdVehicleTrajectory()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    self.validate_spline(msg.spline)
                    v.process_spline_msg(msg.spline )

                elif msg_type == api.CTRL_CMD_VEHICLE_ITINERARY:
                    msg = api.CtrlCmdVehicleItinerary()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    if msg.clear:
                        v.clear_path()
                    self.validate_itinerary(v, msg.trackIDs) # raises InvalidTrackSegmentID if not valid
                    locs = [self.get_trackSeg(id) for id in msg.trackIDs]
                    if locs:
                        v.extend_path(locs)

                elif msg_type == api.CTRL_CMD_SWITCH:
                    msg = api.CtrlCmdSwitch()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    ts = self.get_trackSeg(msg.trackID)
                    next_ts = self.get_trackSeg(msg.nextID)

                    # If not a valid track segment to switch to or it's already
                    # switching, then complain.
                    if next_ts not in common.digraph.neighbors(ts) or ts.next_loc is None:
                        raise common.InvalidTrackSegID, msg.nextID

                    ts.switch(next_ts, msgID)

                elif msg_type == api.CTRL_CMD_PASSENGERS_EMBARK:
                    msg = api.CtrlCmdPassengersEmbark()
                    msg.MergeFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    b = self.get_berth(msg.sID, msg.platformID, msg.berthID)
                    passengers = map(self.get_passenger, msg.passengerIDs)
                    if b.loading and not b.is_busy():
                        b.embark(v, passengers, msg, msgID)
                    else:
                        raise common.InvalidBerthID, b

                elif msg_type == api.CTRL_CMD_PASSENGERS_DISEMBARK:
                    msg = api.CtrlCmdPassengersDisembark()
                    msg.MergeFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    b = self.get_berth(msg.sID, msg.platformID, msg.berthID)
                    passengers = map(self.get_passenger, msg.passengerIDs)
                    if b.unloading and not b.is_busy():
                        b.disembark(v, passengers, msg, msgID)
                    else:
                        raise common.InvalidBerthID, b

                elif msg_type == api.CTRL_CMD_PASSENGER_WALK:
                    msg = api.CtrlCmdPassengerWalk()
                    msg.MergeFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    pax = self.get_passenger(msg.passengerID)
                    origin = self.get_station(msg.origin_stationID)
                    if origin is not pax.loc:
                        raise common.InvalidStationID, msg.origin_stationID
                    dest = self.get_station(msg.dest_stationID)
                    travel_time = msg.travel_time
                    if travel_time < 0:
                        raise common.InvalidTime, travel_time
                    pax.walk(origin, dest, travel_time, msg, msgID)

                elif msg_type == api.CTRL_CMD_STORAGE_ENTER:
                    msg = api.CtrlCmdStorageEnter()
                    msg.MergeFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    b = self.get_berth(msg.sID, msg.platformID, msg.berthID)
                    if b.storage_entrance and not b.is_busy() :
                        b.enter_storage(v, msg, msgID)
                    else:
                        raise common.InvalidBerthID, b.ID

                elif msg_type == api.CTRL_CMD_STORAGE_EXIT:
                    msg = api.CtrlCmdStorageExit()
                    msg.MergeFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    try:
                        model = common.vehicle_models[msg.model_name]
                    except KeyError:
                        raise common.MsgError
                    b = self.get_berth(msg.sID, msg.platformID, msg.berthID)

                    position = msg.position
                    if b.storage_exit \
                            and not b.is_busy() \
                            and position - model.length >= b.start_pos \
                            and position <= b.end_pos:
                        b.exit_storage(position, model.model_name, msg, msgID)
                    else:
                        raise common.InvalidBerthID, b.ID

                # REQUESTS
                elif msg_type == api.CTRL_REQUEST_VEHICLE_STATUS:
                    msg = api.CtrlRequestVehicleStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    resp = api.SimResponseVehicleStatus()
                    resp.msgID = msgID
                    resp.time = Sim.now()
                    v.fill_VehicleStatus(resp.v_status)
                    self.send(api.SIM_RESPONSE_VEHICLE_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_STATION_STATUS:
                    msg = api.CtrlRequestStationStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    stat = self.get_station(msg.sID)
                    resp = api.SimResponseStationStatus()
                    resp.msgID = msgID
                    resp.time = Sim.now()
                    stat.fill_StationStatus(resp.s_status)
                    self.send(api.SIM_RESPONSE_STATION_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_PASSENGER_STATUS:
                    msg = api.CtrlRequestPassengerStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    p = self.get_passenger(msg.pID)
                    resp = api.SimResponsePassengerStatus()
                    resp.msgID = msgID
                    resp.time = Sim.now()
                    p.fill_PassengerStatus(resp.p_status)
                    self.send(api.SIM_RESPONSE_PASSENGER_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_SWITCH_STATUS:
                    msg = api.CtrlRequestSwitchStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    sw = self.get_switch(msg.swID)
                    resp = api.SimResponseSwitchStatus()
                    resp.msgID = msgID
                    resp.time = Sim.now()
                    sw.fill_SwitchStatus(resp.sw_status)
                    self.send(api.SIM_RESPONSE_SWITCH_STATUS, resp)

                elif msg_type == api.CTRL_REQUEST_TRACKSEGMENT_STATUS:
                    msg = api.CtrlRequestTrackSegmentStatus()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    ts = self.get_trackSeg(msg.eID)
                    resp = api.SimResponseTrackSegmentStatus()
                    resp.time = Sim.now()
                    resp.msgID = msgID
                    ts.fill_TrackSegmentStatus(resp.ts_status)
                    self.send(api.SIM_RESPONSE_TRACKSEGMENT_STATUS, resp)


                # NOTIFICATIONS
                elif msg_type == api.CTRL_SETNOTIFY_TIME:
                    msg = api.CtrlSetnotifyTime()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )

                    if utility.time_lt(msg.time, Sim.now()):
                        raise common.InvalidTime, msg.time
                    resp = api.SimNotifyTime()
                    resp.msgID = msgID
                    resp.time = msg.time
                    alarm = common.AlarmClock(msg.time, self.send, api.SIM_NOTIFY_TIME, resp)

                elif msg_type == api.CTRL_SETNOTIFY_VEHICLE_POSITION:
                    msg = api.CtrlSetnotifyVehiclePosition()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    v = self.get_vehicle(msg.vID)
                    # send error message if vehicle has already passed the notification pos
                    # or if pos is beyond the end of the track
                    if utility.dist_lt(v.pos, msg.pos) or utility.dist_gt(msg.pos, v.loc.length):
                        err_resp = api.SimMsgBodyInvalid()
                        err_resp.msgID = msgID
                        self.send(api.SIM_MSG_BODY_INVALID, err_resp)
                    else:
                        v.notify_position(msg, msgID)

                elif msg_type == api.CTRL_SCENARIO_ERROR:
                    msg = api.CtrlScenarioError()
                    msg.ParseFromString(msg_str)
                    self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
                    logging.error("Controller reports the following Scenario Error: " + str(msg))
                    if not common.config_manager.get_disable_gui():
                        common.gui.GetTopWindow().show_message('Controller reports the following Scenario Error:\n' + msg.error_message)
                    main.stop_sim()

                else:
                    resp = api.SimUnimplemented()
                    resp.msgID = msgID
                    self.send(api.SIM_UNIMPLEMENTED, resp)
                    logging.debug("Msg type %d received, but not yet implemented.",
                                  msg_type)

            except common.InvalidMsgType as e:
                logging.exception(e)
                err_msg = api.SimMsgHdrInvalidType()
                err_msg.msgID = msgID
                err_msg.msg_type = msg_type
                self.send(api.SIM_MSG_HDR_INVALID_TYPE, err_msg)
                logging.error("Received invalid msg type: %s", msg_type)
                common.errors += 1
            except common.InvalidTrackSegID as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidId()
                err_msg.msgID = msgID
                err_msg.id_type = api.TRACK_SEGMENT
                err_msg.ID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except common.InvalidSwitchID as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidId()
                err_msg.msgID = msgID
                err_msg.id_type = api.SWITCH
                err_msg.ID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except common.InvalidStationID as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidId()
                err_msg.msgID = msgID
                err_msg.id_type = api.STATION
                err_msg.ID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except common.InvalidPlatformID as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidId()
                err_msg.msgID = msgID
                err_msg.id_type = api.PLATFORM
                err_msg.ID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except common.InvalidBerthID as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidId()
                err_msg.msgID = msgID
                err_msg.id_type = api.BERTH
                err_msg.ID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except common.InvalidVehicleID as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidId()
                err_msg.msgID = msgID
                err_msg.id_type = api.VEHICLE
                err_msg.ID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except common.InvalidPassengerID as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidId()
                err_msg.msgID = msgID
                err_msg.id_type = api.PASSENGER
                err_msg.ID = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_ID, err_msg)
            except common.InvalidTime as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalidTime()
                err_msg.msgID = msgID
                err_msg.time = e.args[0]
                self.send(api.SIM_MSG_BODY_INVALID_TIME, err_msg)
            except common.MsgRangeError as e:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalid()
                err_msg.msgID = msgID
                self.send(api.SIM_MSG_BODY_INVALID, err_msg)
            except common.MsgError:
                logging.exception(e)
                err_msg = api.SimMsgBodyInvalid()
                err_msg.msgID = msgID
                self.send(api.SIM_MSG_BODY_INVALID, err_msg)

        # if packet time is in future, set an alarm to wake up interface
        # at the time, and revive the message then.
        elif msg_time > ms_now():
            common.AlarmClock(msg_time/1000.0, AlarmClock.delayed_msg, (comm_idx, msg_type, msgID, msg_time, msg_str))
        else: # msg_time < ms_now()
            # This check should clearly go away once latency is introduced.
            err = api.SimMsgHdrInvalidTime()
            err.msgID = msgID
            err.msg_time = msg_time
            self.send(api.SIM_MSG_HDR_INVALID_TIME, err)
            logging.error("Time stamp for RCVD message %s type %s in the past. Stamp: %s Now: %s ",
                          msgID, msg_type, msg_time, ms_now())
            common.errors += 1

        return msg_type

    def send(self, msg_type, msg):
        """Send an outgoing message.
        Every message sent is prefixed with:
               msg seperator: 2 bytes, signed -32123
               msg type: 2 bytes, signed
               msg ID: 4 bytes, signed
               msg time: 4 bytes, signed
               msg size: 4 bytes, signed
        The message seperator is always -32123
        The msg type is one of the values found in CtrlMsgType or SimMsgType
        The msg size is the length (in bytes) of the serialized message.
        These values are transmitted in network byte order (Big Endian).

        TEMP: Transmits to all clients (no filtering)
        """
        msgID = common.msg_id_widget.next_id()
        msg_time = ms_now()
        msg_str = msg.SerializeToString()
        msg_size = len(msg_str)
        # big_endian, *std size* and align: short, short, int, int, int
        # short = 2 bytes, int = 4 bytes
        header = struct.pack('>hhiii', api.MSG_SEP, msg_type, msgID,
                             msg_time, msg_size)
        message = header + msg_str

        # TODO: Only send to consumers that are subscribed to that message type
        # and/or message target.
        for q in self.sendQs:
            q.put(message)

        if self.passive() and not common.sim_ended:
            # Reset the resume_list to all False
            self.resume_list[:] = [False] * len(self.resume_list)
            # Reactivate the ControlInterface, placing it at the end of the
            # list of events to run.
            Sim.reactivate(self, prior=False)

        if self.log != os.devnull:
            print >> self.log, "Now:%s, SENT, %s, MsgType:%d, MsgID:%d, MsgTime:%d" %\
                  (time.time(), msg.DESCRIPTOR.name, msg_type, msgID, msg_time)
            text_format.PrintMessage(msg, out=self.log)
            print >> self.log, "--"

    def log_rcvd_msg(self, msg_type, msgID, msg_time, msg):
        # log msg
        if self.log != os.devnull:
            print >> self.log, "Now:%s, RCVD, %s, MsgType:%d, MsgID:%d, MsgTime:%d" %\
                  (time.time(), msg.DESCRIPTOR.name, msg_type, msgID, msg_time)
            text_format.PrintMessage(msg, out=self.log)
            print >> self.log, "--"

    def get_trackSeg(self, ID):
        """"Return the TrackSegment object specified by the ID.
        Raise InvalidTrackSegID if not found."""
        try:
            return common.track_segments[ID]
        except KeyError:
            raise common.InvalidTrackSegID, ID

    def get_station(self, ID):
        """"Return the station object specified by the ID.
        Raise InvalidStationID if not found."""
        try:
            s = common.stations[ID]
        except KeyError:
            raise common.InvalidStationID, ID
        return s

    def get_vehicle(self, ID):
        """Return the vehicle object specified by the ID.
        Raise InvalidVehicleID if not found."""
        try:
            v = common.vehicles[ID]
        except KeyError:
            raise common.InvalidVehicleID, ID
        return v

    def get_passenger(self, ID):
        """"Return the passenger object specified by the ID.
        Raise InvalidPassengerID if not found."""
        try:
            pax = common.passengers[ID]
        except KeyError:
            raise common.InvalidPassengerID, ID
        return pax

    def get_berth(self, station_id, platform_id, berth_id):
        station = self.get_station(station_id)
        try:
            platform = station.platforms[platform_id]
        except KeyError:
            raise common.InvalidPlatformID, platform_id
        try:
            berth = platform.berths[berth_id]
        except KeyError:
            raise common.InvalidBerthID, berth_id
        return berth

    def validate_spline(self, spline):
        """A simple check that the message data is not malformed.
        See: vehicle._validate_spline for more thorough testing of the resulting
             cubic_spline.CubicSpline object.
        """
        assert isinstance(spline, api.Spline)
        # Check that the times for the spline are in non-decreasing order
        if len(spline.times) == 0:
            raise common.MsgTime

        for t1, t2 in pairwise(spline.times):
            if t2 < t1:
                logging.info("T=%4.3f Spline times are not in non-decreasing order: %s",
                             Sim.now(), spline.times)
                raise common.MsgTime

    def validate_itinerary(self, vehicle, ids):
        if len(ids) == 0:
            return
        latest_loc = vehicle.loc
        for id in ids:
            loc = self.get_trackSeg(id)
            neighbors = common.digraph.neighbors(latest_loc)
            if loc not in neighbors:
                raise common.InvalidTrackSegID, ids[0]
            latest_loc = loc

    def fill_TotalStatus(self, ts, msgID=None):
        """Fills a api.TotalStatus message with current data.

        ts: an empty api.TotalStatus message
        msgID: The msgID that this is a response to, if any."""
        if msgID:
            ts.msgID = msgID
        for v in common.vehicles.itervalues():
            v_status = api.VehicleStatus()
            v.fill_VehicleStatus(v_status)
            ts.v_statuses.append(v_status)
        # send track segment status for all track segments
        for ts in common.track_segments.itervalues():
            ts_status = api.TrackSegmentStatus()
            ts.fill_TrackSegmentStatus(ts_status)
            ts.t_statuses.append(ts_status)
        # send Switch status msgs
        for switch in common.switch_list:
            sw_status = api.SwitchStatus()
            switch.fill_SwitchStatus(sw_status)
            ts.sw_statuses.append(sw_status)
        # send Station status msgs and summary msgs
        for station in common.stations.itervalues():
            s_status = api.StationStatus()
            station.fill_StationStatus(s_status)
            ts.s_statuses.append(s_status)

    def make_SimGreeting(self):
        greeting = api.SimGreeting()
        greeting.sim_end_time = common.config_manager.get_sim_end_time()
        scenario_file = open(common.config_manager.get_scenario_path())
        scenario_xml = scenario_file.read()
        scenario_file.close()
        greeting.scenario_xml = scenario_xml;
        return greeting

def ms_now():
    """Return Sim.now() in integer form, rounded to milliseconds. This is the
    form that is used for communication with the Control modules."""
    return int(round(Sim.now()*1000))
