import optparse
import socket
import logging
import struct
from sys import stderr

import google.protobuf.text_format as text_format

import pyprt.shared.api_pb2 as api

def main():
    options_parser = optparse.OptionParser(usage="usage: %prog [options]")
    options_parser.add_option("--logfile", dest="logfile", default="./ctrl.log",
                metavar="FILE", help="Log events to FILE.")
    options_parser.add_option("--comm_logfile", dest="comm_logfile", default="./ctrl_comm.log",
                metavar="FILE", help="Log communication messages to FILE.")
    options_parser.add_option("--server", dest="server", default="localhost",
                help="The IP address of the server (simulator). Default is %default")
    options_parser.add_option("-p", "--port", type="int", dest="port", default=64444,
                help="TCP Port to connect to. Default is: %default")
    options, args = options_parser.parse_args()

    if len(args):
        options_parser.error("Unhandled argument(s): %s" % ' '.join(args))

    ctrl = BaseController(options.logfile, options.comm_logfile)
    ctrl.connect(options.server, options.port)

class BaseController(object):
    def __init__(self, log_path=None, commlog_path=None):
        self.server = None
        self.port = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.next_msgID = 0
        self.last_sim_msgID = -1
        self.last_sim_timestamp = 0 # integer, in milliseconds
        self.current_time = 0.0 # float, in seconds
        self.sim_complete = False
        self.sim_end_time = None

        self.sendQ = list()  #  list contains (msg_type, body) tuples

        self.log = logging.getLogger('log')
        self.comm_log = logging.getLogger('comm_log')
        self.initialize_logging(log_path, commlog_path)

        # The following dictionary was generated using make_msg_handling_stubs.py
        # Instantiates a protobuf message for each message type.
        self.messages = {
            api.SIM_GREETING : api.SimGreeting(),
            api.SIM_START : api.SimStart(),
            api.SIM_END : api.SimEnd(),
            api.SIM_UNIMPLEMENTED : api.SimUnimplemented(),
            api.SIM_COMPLETE_PASSENGERS_EMBARK : api.SimCompletePassengersEmbark(),
            api.SIM_COMPLETE_PASSENGERS_DISEMBARK : api.SimCompletePassengersDisembark(),
            api.SIM_COMPLETE_PASSENGER_WALK : api.SimCompletePassengerWalk(),
            api.SIM_COMPLETE_SWITCH : api.SimCompleteSwitch(),
            api.SIM_COMPLETE_STORAGE_ENTER : api.SimCompleteStorageEnter(),
            api.SIM_COMPLETE_STORAGE_EXIT : api.SimCompleteStorageExit(),
            api.SIM_RESPONSE_VEHICLE_STATUS : api.SimResponseVehicleStatus(),
            api.SIM_RESPONSE_STATION_STATUS : api.SimResponseStationStatus(),
            api.SIM_RESPONSE_PASSENGER_STATUS : api.SimResponsePassengerStatus(),
            api.SIM_RESPONSE_SWITCH_STATUS : api.SimResponseSwitchStatus(),
            api.SIM_RESPONSE_TRACK_STATUS : api.SimResponseTrackStatus(),
            api.SIM_RESPONSE_TOTAL_STATUS : api.SimResponseTotalStatus(),
            api.SIM_NOTIFY_VEHICLE_POSITION : api.SimNotifyVehiclePosition(),
            api.SIM_NOTIFY_VEHICLE_ARRIVE : api.SimNotifyVehicleArrive(),
            api.SIM_NOTIFY_VEHICLE_EXIT : api.SimNotifyVehicleExit(),
            api.SIM_NOTIFY_VEHICLE_STOPPED : api.SimNotifyVehicleStopped(),
            api.SIM_NOTIFY_VEHICLE_SPEEDING : api.SimNotifyVehicleSpeeding(),
            api.SIM_NOTIFY_VEHICLE_COLLISION : api.SimNotifyVehicleCollision(),
            api.SIM_NOTIFY_VEHICLE_CRASH : api.SimNotifyVehicleCrash(),
            api.SIM_NOTIFY_PASSENGER_EMBARK_START : api.SimNotifyPassengerEmbarkStart(),
            api.SIM_NOTIFY_PASSENGER_EMBARK_END : api.SimNotifyPassengerEmbarkEnd(),
            api.SIM_NOTIFY_PASSENGER_DISEMBARK_START : api.SimNotifyPassengerDisembarkStart(),
            api.SIM_NOTIFY_PASSENGER_DISEMBARK_END : api.SimNotifyPassengerDisembarkEnd(),
            api.SIM_NOTIFY_TIME : api.SimNotifyTime(),
            api.SIM_EVENT_TRACK_DISABLED : api.SimEventTrackDisabled(),
            api.SIM_EVENT_TRACK_REENABLED : api.SimEventTrackReenabled(),
            api.SIM_EVENT_SWITCH_DISABLED : api.SimEventSwitchDisabled(),
            api.SIM_EVENT_SWITCH_REENABLED : api.SimEventSwitchReenabled(),
            api.SIM_EVENT_STATION_DISABLED : api.SimEventStationDisabled(),
            api.SIM_EVENT_STATION_REENABLED : api.SimEventStationReenabled(),
            api.SIM_EVENT_VEHICLE_DISABLED : api.SimEventVehicleDisabled(),
            api.SIM_EVENT_VEHICLE_REENABLED : api.SimEventVehicleReenabled(),
            api.SIM_EVENT_PASSENGER_CREATED : api.SimEventPassengerCreated(),
            api.SIM_EVENT_PASSENGER_CHANGEDEST : api.SimEventPassengerChangedest(),
            api.SIM_MSG_HDR_INVALID_SEPARATOR : api.SimMsgHdrInvalidSeparator(),
            api.SIM_MSG_HDR_INVALID_TYPE : api.SimMsgHdrInvalidType(),
            api.SIM_MSG_HDR_INVALID_ID : api.SimMsgHdrInvalidId(),
            api.SIM_MSG_HDR_INVALID_TIME : api.SimMsgHdrInvalidTime(),
            api.SIM_MSG_HDR_INVALID_SIZE : api.SimMsgHdrInvalidSize(),
            api.SIM_MSG_BODY_INVALID : api.SimMsgBodyInvalid(),
            api.SIM_MSG_BODY_INVALID_TIME : api.SimMsgBodyInvalidTime(),
            api.SIM_MSG_BODY_INVALID_ID : api.SimMsgBodyInvalidId()
        }

        # The following dictionary was generated using make_msg_handling_stubs.py
        # Maps message types to message handling functions.
        self.msg_handlers = {
            api.SIM_GREETING : self.on_SIM_GREETING,
            api.SIM_START : self.on_SIM_START,
            api.SIM_END : self.on_SIM_END,
            api.SIM_UNIMPLEMENTED : self.on_SIM_UNIMPLEMENTED,
            api.SIM_COMPLETE_PASSENGERS_EMBARK : self.on_SIM_COMPLETE_PASSENGERS_EMBARK,
            api.SIM_COMPLETE_PASSENGERS_DISEMBARK : self.on_SIM_COMPLETE_PASSENGERS_DISEMBARK,
            api.SIM_COMPLETE_PASSENGER_WALK : self.on_SIM_COMPLETE_PASSENGER_WALK,
            api.SIM_COMPLETE_SWITCH : self.on_SIM_COMPLETE_SWITCH,
            api.SIM_COMPLETE_STORAGE_ENTER : self.on_SIM_COMPLETE_STORAGE_ENTER,
            api.SIM_COMPLETE_STORAGE_EXIT : self.on_SIM_COMPLETE_STORAGE_EXIT,
            api.SIM_RESPONSE_VEHICLE_STATUS : self.on_SIM_RESPONSE_VEHICLE_STATUS,
            api.SIM_RESPONSE_STATION_STATUS : self.on_SIM_RESPONSE_STATION_STATUS,
            api.SIM_RESPONSE_PASSENGER_STATUS : self.on_SIM_RESPONSE_PASSENGER_STATUS,
            api.SIM_RESPONSE_SWITCH_STATUS : self.on_SIM_RESPONSE_SWITCH_STATUS,
            api.SIM_RESPONSE_TRACK_STATUS : self.on_SIM_RESPONSE_TRACK_STATUS,
            api.SIM_RESPONSE_TOTAL_STATUS : self.on_SIM_RESPONSE_TOTAL_STATUS,
            api.SIM_NOTIFY_VEHICLE_POSITION : self.on_SIM_NOTIFY_VEHICLE_POSITION,
            api.SIM_NOTIFY_VEHICLE_ARRIVE : self.on_SIM_NOTIFY_VEHICLE_ARRIVE,
            api.SIM_NOTIFY_VEHICLE_EXIT : self.on_SIM_NOTIFY_VEHICLE_EXIT,
            api.SIM_NOTIFY_VEHICLE_STOPPED : self.on_SIM_NOTIFY_VEHICLE_STOPPED,
            api.SIM_NOTIFY_VEHICLE_COLLISION : self.on_SIM_NOTIFY_VEHICLE_COLLISION,
            api.SIM_NOTIFY_VEHICLE_SPEEDING : self.on_SIM_NOTIFY_VEHICLE_SPEEDING,
            api.SIM_NOTIFY_VEHICLE_CRASH : self.on_SIM_NOTIFY_VEHICLE_CRASH,
            api.SIM_NOTIFY_PASSENGER_EMBARK_START : self.on_SIM_NOTIFY_PASSENGER_EMBARK_START,
            api.SIM_NOTIFY_PASSENGER_EMBARK_END : self.on_SIM_NOTIFY_PASSENGER_EMBARK_END,
            api.SIM_NOTIFY_PASSENGER_DISEMBARK_START : self.on_SIM_NOTIFY_PASSENGER_DISEMBARK_START,
            api.SIM_NOTIFY_PASSENGER_DISEMBARK_END : self.on_SIM_NOTIFY_PASSENGER_DISEMBARK_END,
            api.SIM_NOTIFY_TIME : self.on_SIM_NOTIFY_TIME,
            api.SIM_EVENT_TRACK_DISABLED : self.on_SIM_EVENT_TRACK_DISABLED,
            api.SIM_EVENT_TRACK_REENABLED : self.on_SIM_EVENT_TRACK_REENABLED,
            api.SIM_EVENT_SWITCH_DISABLED : self.on_SIM_EVENT_SWITCH_DISABLED,
            api.SIM_EVENT_SWITCH_REENABLED : self.on_SIM_EVENT_SWITCH_REENABLED,
            api.SIM_EVENT_STATION_DISABLED : self.on_SIM_EVENT_STATION_DISABLED,
            api.SIM_EVENT_STATION_REENABLED : self.on_SIM_EVENT_STATION_REENABLED,
            api.SIM_EVENT_VEHICLE_DISABLED : self.on_SIM_EVENT_VEHICLE_DISABLED,
            api.SIM_EVENT_VEHICLE_REENABLED : self.on_SIM_EVENT_VEHICLE_REENABLED,
            api.SIM_EVENT_PASSENGER_CREATED : self.on_SIM_EVENT_PASSENGER_CREATED,
            api.SIM_EVENT_PASSENGER_CHANGEDEST : self.on_SIM_EVENT_PASSENGER_CHANGEDEST,
            api.SIM_MSG_HDR_INVALID_SEPARATOR : self.on_SIM_MSG_HDR_INVALID_SEPARATOR,
            api.SIM_MSG_HDR_INVALID_TYPE : self.on_SIM_MSG_HDR_INVALID_TYPE,
            api.SIM_MSG_HDR_INVALID_ID : self.on_SIM_MSG_HDR_INVALID_ID,
            api.SIM_MSG_HDR_INVALID_TIME : self.on_SIM_MSG_HDR_INVALID_TIME,
            api.SIM_MSG_HDR_INVALID_SIZE : self.on_SIM_MSG_HDR_INVALID_SIZE,
            api.SIM_MSG_BODY_INVALID : self.on_SIM_MSG_BODY_INVALID,
            api.SIM_MSG_BODY_INVALID_TIME : self.on_SIM_MSG_BODY_INVALID_TIME,
            api.SIM_MSG_BODY_INVALID_ID : self.on_SIM_MSG_BODY_INVALID_ID
    }

    def initialize_logging(self, log_path, commlog_path):
        # Set up the main log file
        if log_path is None:
            handler = logging.StreamHandler()
        else:
            handler = logging.FileHandler(log_path, 'w')
        formatter = logging.Formatter('%(filename)10s %(funcName)20s line:%(lineno)d' \
                            ' walltime:%(relativeCreated)dms %(levelname)8s %(message)s')
        handler.setFormatter(formatter)
        handler.setLevel(logging.DEBUG)
        self.log.setLevel(logging.DEBUG)
        self.log.addHandler(handler)

        # Set up the comm log file
        if commlog_path is None:
            handler = logging.StreamHandler()
        else:
            handler = logging.FileHandler(commlog_path, 'w')
        formatter = logging.Formatter('%(message)s---- walltime: %(relativeCreated)dms')
        handler.setFormatter(formatter)
        handler.setLevel(logging.DEBUG)
        self.comm_log.setLevel(logging.DEBUG)
        self.comm_log.addHandler(handler)

    def connect(self, server, port):
        self.server = server
        self.port = port
        try:
            self.sock.connect( (server, port) )
        except socket.error, msg:
            stderr.write("Failed to connect to %s:%d\n" % (server, port))
            stderr.write(str(msg))
            return
        self.log.info("Connected to server: %s:%d", server, port)
        self.talk()

    def disconnect(self):
        self.sock.close()
        self.log.info("Disconnected from server: %s:%d", self.server, self.port)

    def receive(self):
        """Receive a packet from sim and returns a tuple:
            msg_type, msgID, msg_time, body

        Raises an exception if msg_sep or msg_size is invalid.
        """
        # get new msgs
        header = self.sock.recv(api.MSG_HEADER_SIZE)
        while len(header) < api.MSG_HEADER_SIZE:
            chunk = self.sock.recv(api.MSG_HEADER_SIZE - len(header))
            header += chunk

        # unpack header
        msg_sep, msg_type, msgID, msg_time, msg_size \
                = struct.unpack('>hhiii', header)

        # TODO: Subtype Exception,  and handle
        if msg_sep != api.MSG_SEP:
            raise Exception, "Msg separator was incorrect: %d" % msg_sep
        if msg_size < 0:
            raise Exception, "Negative msg_size received"

        # Get the message body
        body_parts = []
        while msg_size:
            frag = self.sock.recv(msg_size)
            msg_size -= len(frag)
            body_parts.append(frag)
        body = ''.join(body_parts)

        self.last_sim_timestamp = max(msg_time, self.last_sim_timestamp)
        self.last_sim_msgID = max(self.last_sim_msgID, msgID)
        return (msg_type, msgID, msg_time, body)

    def transmit(self):
        """Transmit accumulated outgoing packets.
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
        """
        for msg_type, msg in self.sendQ:
            msgID = self.next_msgID
            self.next_msgID += 1
            msg_str = msg.SerializeToString()
            msg_size = len(msg_str)
            header = struct.pack('>hhiii', api.MSG_SEP, msg_type, msgID,
                                 self.last_sim_timestamp, msg_size)
            self.sock.sendall(header + msg_str)
            self.comm_log.info("SENT, %s, %d, %d, %d\n" %\
                 (msg.DESCRIPTOR.name, msg_type, msgID, self.last_sim_timestamp) + \
                 text_format.MessageToString(msg))

        self.sendQ = list()

    def talk(self):
        """Receive/Transmit loop. Expects that all messages the controller logic
        intends to transmit will be queued up during the receive/handle_msg
        part of the loop."""
        while not self.sim_complete:
            try:
                msg_type, msgID, msg_time, msg_str = self.receive()
            except socket.error:
                self.log.error("socket.error exception")
                return
            self.handle_msg(msg_type, msgID, msg_time, msg_str)
            self.transmit()

    def send(self, msg_type, msg):
        """Buffers the outgoing message for later transmission.
        Takes a msg_type (one of the numeric values from CtrlMsgType
        in api.proto) and a protobuffer message.

        A good place to do validity checking?
        """
        self.sendQ.append( (msg_type, msg) )


    def handle_msg(self, msg_type, msgID, msg_time, msg_str):
        """May be overridden by a subclass. Using a dictionary containing
        message handling functions is merely a suggested approach.
        Note that this implementation reuses the message objects, and is not
        reentrant.
        Updates self.current_time with the new (floating point) time whenever
        a message is received that contains time stamp info.
        """
        msg = self.messages[msg_type]
        msg.ParseFromString(msg_str) # clear old data, and deserialize string into msg
        self.log_rcvd_msg(msg_type, msgID, msg_time, msg)
        try:
            if msg.HasField('time'):
                self.current_time = msg.time
        except ValueError:
            pass
        self.msg_handlers[msg_type](msg, msgID, msg_time) # call the msg handler
        self.send_resume()

    def log_rcvd_msg(self, msg_type, msgID, msg_time, msg):
        self.comm_log.info("RCVD, %s, %d, %d, %d\n" %\
                 (msg.DESCRIPTOR.name, msg_type, msgID, msg_time) + \
                 text_format.MessageToString(msg))

    def send_resume(self):
        resume = api.CtrlResume()
        resume.last_sim_msgID = self.last_sim_msgID
        self.send(api.CTRL_RESUME, resume)

    def on_SIM_GREETING(self, msg, msgID, msg_time):
        self.log.info("Sim Greeting message received.")
        self.sim_end_time = msg.sim_end_time

    def on_SIM_START(self, msg, msgID, msg_time):
        self.log.info("Sim Start message received.")

    def on_SIM_END(self, msg, msgID, msg_time):
        self.log.info("Sim End message received.")
        self.sim_complete = True

    def on_SIM_UNIMPLEMENTED(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_COMPLETE_PASSENGERS_EMBARK(self, msg, msgID, msg_time):
        pass

    def on_SIM_COMPLETE_PASSENGERS_DISEMBARK(self, msg, msgID, msg_time):
        pass

    def on_SIM_COMPLETE_PASSENGER_WALK(self, msg, msgID, msg_time):
        pass

    def on_SIM_COMPLETE_SWITCH(self, msg, msgID, msg_time):
        pass

    def on_SIM_COMPLETE_STORAGE_ENTER(self, msg, msgID, msg_time):
        pass

    def on_SIM_COMPLETE_STORAGE_EXIT(self, msg, msgID, msg_time):
        pass

    def on_SIM_RESPONSE_VEHICLE_STATUS(self, msg, msgID, msg_time):
        pass

    def on_SIM_RESPONSE_STATION_STATUS(self, msg, msgID, msg_time):
        pass

    def on_SIM_RESPONSE_PASSENGER_STATUS(self, msg, msgID, msg_time):
        pass

    def on_SIM_RESPONSE_SWITCH_STATUS(self, msg, msgID, msg_time):
        pass

    def on_SIM_RESPONSE_TRACK_STATUS(self, msg, msgID, msg_time):
        pass

    def on_SIM_RESPONSE_TOTAL_STATUS(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_POSITION(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_ARRIVE(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_EXIT(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_STOPPED(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_SPEEDING(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_READY_LOAD(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_COLLISION(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_VEHICLE_CRASH(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_PASSENGER_EMBARK_START(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_PASSENGER_EMBARK_END(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_PASSENGER_DISEMBARK_START(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_PASSENGER_DISEMBARK_END(self, msg, msgID, msg_time):
        pass

    def on_SIM_NOTIFY_TIME(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_TRACK_DISABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_TRACK_REENABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_SWITCH_DISABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_SWITCH_REENABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_STATION_DISABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_STATION_REENABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_VEHICLE_DISABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_VEHICLE_REENABLED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_PASSENGER_CREATED(self, msg, msgID, msg_time):
        pass

    def on_SIM_EVENT_PASSENGER_CHANGEDEST(self, msg, msgID, msg_time):
        pass

    def on_SIM_MSG_HDR_INVALID_SEPARATOR(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_MSG_HDR_INVALID_TYPE(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_MSG_HDR_INVALID_ID(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_MSG_HDR_INVALID_TIME(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_MSG_HDR_INVALID_SIZE(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_MSG_BODY_INVALID(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_MSG_BODY_INVALID_TIME(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

    def on_SIM_MSG_BODY_INVALID_ID(self, msg, msgID, msg_time):
        raise Exception("Message rejected by Sim")

if __name__ == '__main__':
    main()