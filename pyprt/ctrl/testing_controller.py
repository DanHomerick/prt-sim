import optparse

from base_controller import BaseController
import pyprt.shared.api_pb2 as api

class TestingController(BaseController):
    def __init__(self, log_path, commlog_path):
        super(TestingController, self).__init__(log_path, commlog_path)

    def on_SIM_START(self, msg, msgID, msg_time):
        """Request a vehicle from storage """
        cmd = api.CtrlCmdStorageExit()
        cmd.sID = 0
        cmd.platformID = 1
        cmd.berthID = 0
        cmd.position = 4.5
        cmd.model_name = 'PRT_DEFAULT'
        self.send(api.CTRL_CMD_STORAGE_EXIT, cmd)

    def on_SIM_COMPLETE_STORAGE_ENTER(self, msg, msgID, msg_time):
        """Take the vehicle back out of storage"""
        cmd = api.CtrlCmdStorageExit()
        cmd.sID = 0
        cmd.platformID = 1
        cmd.berthID = 0
        cmd.position = 4.2
        cmd.model_name = 'PRT_DEFAULT'
        self.send(api.CTRL_CMD_STORAGE_EXIT, cmd)

    def on_SIM_COMPLETE_STORAGE_EXIT(self, msg, msgID, msg_time):
        """Put the vehicle back into storage"""
        cmd = api.CtrlCmdStorageEnter()
        cmd.vID = 1
        cmd.sID = 0
        cmd.platformID = 1
        cmd.berthID = 0
        self.send(api.CTRL_CMD_STORAGE_ENTER, cmd)

def main(options):
    ctrl = TestingController(options.logfile, options.comm_logfile)
    ctrl.connect(options.server, options.port)

if __name__ == '__main__':
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

    if len(args) != 0:
        options_parser.error("Expected zero positional arguments. Received: %s" % ' '.join(args))

    main(options)