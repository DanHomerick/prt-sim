from __future__ import division # floating division by default

import optparse

import pyprt.shared.api_pb2 as api
import base_controller
import trajectory_solver
import pyprt.shared.cubic_spline as cubic_spline

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

    ctrl = DebugController(options.logfile, options.comm_logfile)
    ctrl.connect(options.server, options.port)

class DebugController(base_controller.BaseController):
    V_MIN = 0
    V_MAX = 20
    A_MIN = -5
    A_MAX = 5
    J_MIN = -2.5
    J_MAX = 2.5

    def __init__(self, log_path=None, commlog_path=None):
        super(DebugController, self).__init__(log_path, commlog_path)
        self.traj_solver = trajectory_solver.TrajectorySolver(self.V_MAX, self.A_MAX, self.J_MAX, self.V_MIN, self.A_MIN, self.J_MIN)

    ### Overriden message handlers ###
    def on_SIM_START(self, msg_type, msgID, msg_time, msg_str):
        msg = api.SimStart()
        msg.MergeFromString(msg_str)
        self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
        self.log.info("Sim Start message received.")

        v_request = api.CtrlRequestVehicleStatus()
        v_request.vID = 0
        self.send(api.CTRL_REQUEST_VEHICLE_STATUS, self.sim_time, v_request)
        self.send_resume()

    def on_SIM_RESPONSE_VEHICLE_STATUS(self, msg_type, msgID, msg_time, msg_str):
        msg = api.SimResponseVehicleStatus()
        msg.MergeFromString(msg_str)
        self.log_rcvd_msg( msg_type, msgID, msg_time, msg )

        initial_knot = cubic_spline.Knot(msg.v_status.nose_pos,
                                         msg.v_status.vel,
                                         msg.v_status.accel,
                                         msg_time/1000) # ms -> seconds
        final_knot = cubic_spline.Knot(msg.v_status.nose_pos + 200, 0, 0, None)
        spline = self.traj_solver.target_position(initial_knot, final_knot)

        inf = 1E10000
        spline.append(cubic_spline.Knot(msg.v_status.nose_pos + 200, 0, 0, inf)) # stay stopped here

        traj_cmd = api.CtrlCmdVehicleTrajectory()
        traj_cmd.vID = msg.v_status.vID
        self.fill_spline_msg(spline, traj_cmd.spline)
        self.send(api.CTRL_CMD_VEHICLE_TRAJECTORY, self.sim_time, traj_cmd)

        # Control the vehicle's path using a vehicle-based switching model
#        itin_cmd = api.CtrlCmdVehicleItinerary()
#        itin_cmd.vID = msg.v_status.vID
#        itin_cmd.tsIDs.extend([9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 1])
#        self.send(api.CTRL_CMD_VEHICLE_ITINERARY, self.sim_time, itin_cmd)

        # Control the vehicle's path using a track-based switching model
        sw_cmd = api.CtrlCmdSwitch()
        sw_cmd.tsID = msg.v_status.nose_locID # current ts
        sw_cmd.nextID = 9 # station entrance
        self.send(api.CTRL_CMD_SWITCH, self.sim_time, sw_cmd)

        self.send_resume()

if __name__ == '__main__':
    main()