"""Autogenerates python code. Creates a message handler for each 'SIM_XXX'
message found in the *.proto file. Also creates a dictionary that maps each
message type (an enumeration) to it's handler. The generated code is not
magically inserted anywhere, it needs to be copy and pasted to the desired
locations."""

import sys

if len(sys.argv) != 3:    
    print "usage: %prog PROTO OUTPUT"
    sys.exit(1)

print sys.argv
input = open(sys.argv[1], 'rU')
output = open(sys.argv[2], 'w')

faux_dict = '    self.msg_handlers = {\n'
for line in input:
    try:
        line, junk = line.split('=')
    except:
        continue
    
    line = line.strip()
    if line[:3] != 'SIM':
        continue
    
    Title_Case = line.title()
    TitleCase = Title_Case.replace('_', '')

    faux_dict += '      api.%s : self.on_%s,\n' %(line, line)

    print >> output, """        def on_%s(self, msg_type, msgID, msg_time, msg_str):
            msg = api.%s()
            msg.MergeFromString(msg_str)
            self.log_rcvd_msg( msg_type, msgID, msg_time, msg )
            self.send_resume()
            """ % (line, TitleCase)

faux_dict += '}'
print >> output, faux_dict