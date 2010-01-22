from __future__ import division # int division may result in a float
"""Traffic data is most commonly available as a table of road segments, with
traffic volumes in each direction for the segment.

This script attempts to create a reasonable set of trips matching the provided
segment usage. The average trip length is unknown. At one extreme, all vehicles
travel no more than one segment -- this would make it easy to generate
a set of trips that perfectly matched the inputted traffic volumes, but would
be quite unrealistic.

Instead what is done is that the shortest path from each node to
every other node is calculated. Each path is weighted by the average
traffic volume for the edges the path uses. Paths are chosen randomly
(based on their weighting) until the total traffic volume has been
generated. In some cases the traffic on a particular edge may deviate
significantly from the requested amount. Should provide a reasonable
mixture of src / dest trips.

Trip start times follow a beta distribution, where the shape parameters are
specified as arguments. 
"""

import sys
import random
import networkx as NX
import numpy
from optparse import OptionParser

optpar = OptionParser(usage="usage: %prog [options] input-file")

optpar.set_usage("""usage: %prog [options] input-file

Must supply a comma separated input-file with lines consisting of
    Src, Dest, Volume
    
    Ex:
    A, B, 100
    B, A, 120
    B, C, 50
    C, A, 80

    Indicates that the traffic volume on piece of track connecting stations A
    and B should be 100, and the traffic on the separate track connecting B to
    A should be 120. Additionally, the traffic on the track between B and C
    should be 50 and from C to A should be 80. In this example, there is no
    track connecting C to B.
    
    Only supply entries for immediately adjacent stations. The network shape
    will be deduced from the edges, and passengers will be generated such that
    all possible shortest paths between all nodes are used."""
)

optpar.add_option("-t", "--time-dist", action="store", nargs=2, dest="beta",
                  default=(1,1), type='float',
                  help = "Two shape parameters for a beta distribution "\
                  "controlling the passenger start times. Default is 1 1 "\
                  "(uniform)."
                  )
optpar.add_option("-e", "--endtime", default=100.0, type='float',
                  help="Latest start time allowed for any passenger.")
optpar.add_option("-o", "--output-file", default='-', dest="out",
                  help="Filename for output. Use '-' for stdout (default).")
optpar.add_option("-m", "--matrix",
                  help="Filename for comma separated Origin-Destination matrix.")
optpar.add_option("-a", "--append", action="store_true", default=False,
                  help="Append to the outfile (and matrix "\
                  "file if specified) rather than overwriting.")
options, args = optpar.parse_args()

if len(args) != 1:
    print optpar.error("Must supply an input-file name.")
inputfile = open(args[0], 'Ur')
if options.out == '-':
    outfile = sys.stdout
else:
    if options.append:
        outfile = open(options.out, 'a')
    else:
        outfile = open(options.out, 'w')

if options.matrix:
    if options.append:
        matrixfile = open(options.matrix, 'a')
    else:
        matrixfile = open(options.matrix, 'w')

# Build graph
G = NX.DiGraph()
total_traffic = 0
target_traffic = {} # target amount of per link traffic
for line in inputfile:
    n1, n2, traffic = [entry.strip() for entry in line.split(',')]
    
    # If node labels are integers, convert them so that sorting will be correct.
    try: n1 = int(n1)
    except ValueError: pass
    try: n2 = int(n2)
    except ValueError: pass
    
    traffic = int(traffic)
    key = (n1,n2)
    G.add_edge(n1, n2)
    total_traffic += traffic
    target_traffic[key] = traffic

# Find the shortest path for every pair of nodes.
all_paths = [NX.shortest_path(G, n1, n2)
             for n1 in G.nodes()
             for n2 in G.nodes()
             if n1 is not n2]

# Remove any False paths -- cases for which there was no path.
all_paths = [p for p in all_paths if p]

# generate mapping from node name (str) to an index in the OD matrix
nodes = G.nodes()
nodes.sort()               
node2index = {}
for idx, n in enumerate(nodes):
    node2index[n] = idx

# build dictionary with edges as keys and paths as values.
edge2paths = dict()
for e in G.edges():
    edge2paths[e] = []
for path in all_paths:
    for n1, n2 in zip(path[:-1], path[1:]):
        e = (n1, n2)
        edge2paths[e].append(path)

# build dictionary with edges as keys and weights as values
edge2weights = [(e, target_traffic[e]) for e in G.edges()]
edge2weights = dict(edge2weights) # tranform from list of tuples to dict. 

od_matrix = numpy.zeros( (len(nodes), len(nodes)) ) # n x n matix
edge_counts = dict()
path_len_count = {}
results = []

total_weight = sum(edge2weights.itervalues())
while total_weight > 0:
    # choose edge (based on current weights)
    rnum = random.randint(1, total_weight)
    subtotal = 0
    for e, w in edge2weights.iteritems():
        subtotal += w
        if rnum <= subtotal:      # picked current edge
            # choose a path that uses my selected edge
            path = random.sample(edge2paths[e], 1)[0]
            results.append(path)
            od_matrix[node2index[path[0]]][node2index[path[-1]]] += 1            

            # Redundant info, but *shrug*
            for n1, n2 in zip(path[:-1], path[1:]):
                key = (n1, n2)
                try:
                    edge_counts[key] += 1
                except KeyError:
                    edge_counts[key] = 1

            # keep statistics on lengths of paths used
            try:
                path_len_count[len(path)-1] += 1
            except KeyError:
                path_len_count[len(path)-1] = 1

            
            # update weight on every edge that used the path
            for n1, n2 in zip(path[:-1], path[1:]):
                e2 = (n1, n2)
                edge2weights[e2] -= 1

                # if this edge is 'used up' remove all paths that use it.
                if edge2weights[e2] == 0:
                    for p in edge2paths[e2]:
                        other_edges = G.edges()
                        other_edges.remove(e2)
                        for oe in other_edges:
                            new_paths = edge2paths[oe]
                            try:
                                new_paths.remove(p)
                                edge2paths[oe] = new_paths
                            except ValueError:
                                pass
        
            break
        
    total_weight = sum(edge2weights.itervalues())

start_times = []
for i in xrange(len(results)):
    start_times.append(random.betavariate(options.beta[0], options.beta[1])*options.endtime)
start_times.sort()

print >> outfile, "# Passenger file autogenerated with: \n# 'python %s'" \
          % (' '.join(sys.argv))
print >> outfile, "\n# Edge Counts:  Actual,   Target"
actual_total = 0
for k, v in edge_counts.items():
    print >> outfile, "# %s: %d,\t%d" % (k, v, target_traffic[k])
    actual_total += v

print >> outfile, "\n# Actual Total: %d, Target Total: %d" \
        % (actual_total, total_traffic)

print >> outfile, "\n# Path length frequencies:"    
for k, v in path_len_count.items():
    print >> outfile, "#", k, v

print >> outfile, "\n# Total passengers: %d arriving in %s seconds" \
          % (len(results), options.endtime)

print >> outfile, "\n# Origin-Destination Matrix (Origin on Y, Dest on X)"
print >> outfile, "# -------------------------"
# print the header
print >> outfile, "# ", " "*5,
for n in nodes:
    print >> outfile, "%5s" % n,
print >> outfile, "\n",
# print the matrix
for row_idx, n in zip(xrange(len(nodes)), nodes):
    print >> outfile, "# ", "%5s" % n,
    for col_idx in xrange(len(nodes)):
        print >> outfile, "%5d" % od_matrix[row_idx][col_idx],
    print >> outfile, "\n",

print >> outfile, "\n# pID\tsStat\tdStat\tsTime\tload\tunload"
for idx, p_t in enumerate(zip(results, start_times)):
    path, time = p_t
    print >> outfile, ("%s\t"*3 + "%.3f") \
          % (idx+1, path[0], path[-1], time)


# output csv version of OD matrix
if options.matrix:
    for row_idx in xrange(len(nodes)):
        for col_idx in xrange(len(nodes)):
            print >> matrixfile, "%5d," % od_matrix[row_idx][col_idx],
        print >> matrixfile, ''
    print >> matrixfile, ''