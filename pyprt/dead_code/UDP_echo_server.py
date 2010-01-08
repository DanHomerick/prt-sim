import SocketServer as SS
import struct

class EchoHandler(SS.BaseRequestHandler):
    def handle(self):
        print "Connected from:", self.client_address
        format = "!3if"
        rdata = struct.unpack(format, self.request[0])
        print rdata
        
#        print self.request[1].sendto(rData, self.client_address)

srv = SS.ThreadingUDPServer(('',8881), EchoHandler)
srv.serve_forever()

