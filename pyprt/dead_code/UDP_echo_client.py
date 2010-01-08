import socket
import struct

from api import *

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg_ID = 0
vsc = pack_packet(msg_ID, BROADCAST, 0.0, CTRL_VEHICLE_SPEED_COMMAND,
                  4, 5, 6, 7)
sock.sendto(vsc, ('localhost', 64444))
sock.close()
