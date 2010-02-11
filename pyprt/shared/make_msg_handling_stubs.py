"""Autogenerates python code. Creates a message handler for each 'SIM_XXX'
message found in the *.proto file. Also creates a dictionary that maps each
message type (an enumeration) to it's handler. The generated code is not
magically inserted anywhere, it needs to be copy and pasted to the desired
locations."""

import sys

if len(sys.argv) != 3:
    print "usage: %prog PROTO OUTPUT"
    sys.exit(1)

input = open(sys.argv[1], 'rU')
output = open(sys.argv[2], 'w')

init_msgs = ['self.messages = {']
handlers = ['self.msg_handlers = {']
stubs = []

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

    init_msgs.append('    api.%s : api.%s(),' % (line, TitleCase))

    handlers.append('    api.%s : self.on_%s,' % (line, line))

    stubs.append("""def on_%s(self, msg, msgID, msg_time):""" % line)
    if 'Invalid' in TitleCase or 'SIM_UNIMPLEMENTED' in line:
        stubs.append('    raise Exception("Message rejected by Sim")')
    else:
        stubs.append('    pass')
    stubs.append('')


init_msgs.append('}')
handlers.append('}')


print >> output, '\n'.join(stubs)
print >> output, '\n'
print >> output, '\n'.join(init_msgs)
print >> output, '\n'
print >> output, '\n'.join(handlers)