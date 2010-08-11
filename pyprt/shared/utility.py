"""General purpose functions, applicable across different projects"""

import itertools
import collections

def pairwise(iterable):
    """s -> (s0,s1), (s1,s2), (s2, s3), ...
    Compatible with Python 2.5+
    """
    a, b = itertools.tee(iterable)
    for elem in b:
        break
    return itertools.izip(a, b)

# Divide resolution by two for rounding.
DIST_RES = 0.1   # 10 cm
DIST_RND = 2
TIME_RES = 0.001
TIME_RND = 3

def same_sign(x, y):
    """Returns true if x and y have the same sign. Zeros are treated as positive."""
    return (x<0) == (y<0)

class deque(collections.deque):
    """An extension of the deque class which includes an insert method."""

    def insert(self, index, item):
        self.rotate(-index)
        self.appendleft(item)
        if index >= 0:
            self.rotate(index)
        else: # negative index used
            self.rotate(index-1)

    def index(self, item):
        # Implementation ASSUMES that deque is implimented as a linked-list,
        # and that rotations are cheap, and that indexing is cheaper when close
        # to 0. Didn't bother timing it.
        index = 0
        while index < len(self):
            if item == self[0]:
                self.rotate(index)
                return index
            else:
                index += 1
                self.rotate(-1)

        self.rotate(index)
        raise ValueError("deque.index(item): item not in deque")


##def find_distant_segs_forward(graph, initial_node, start_dist, end_dist,
##                              nodes=[], starts=[], ends=[]):
##    """Recursively walk the edges of a weighted networkx digraph until start_dist is
##    reached, then adds all nodes encountered until end_dist is reached.
##    Returns a set of nodes"""
##    succs = graph.successors(initial_node)
##    for node in succs:
##        edge_length = graph[initial_node][node]['weight']
##        if start_dist < edge_length:
##            start = max(start_dist, 0)
##            end = min(edge_length, end_dist)
##            try: # don't add duplicates. Use the widest start/end values found
##                idx = nodes.index(node)
##                starts[idx] = min(start, starts[idx])
##                ends[idx] = max(end, ends[idx])
##            except ValueError: # regular case, adding a new node
##                nodes.append(node)
##                starts.append(start)
##                ends.append(end)
##
##        if end_dist >= edge_length:
##            find_distant_segs_forward(graph, node, edge_length,
##                                      end_dist-edge_length, nodes, starts, ends)
##    return nodes, starts, ends

def dist_eql(a, b):
    """Include == test to capture float('inf') == float('inf')"""
    return (True if a == b or round(abs(a-b), DIST_RND) < DIST_RES else False)
def dist_ge(a, b):
    return (True if a > b or dist_eql(a,b) else False)
def dist_le(a, b):
    return (True if a < b or dist_eql(a,b) else False)
def dist_gt(a, b):
    return (True if a > b and not dist_eql(a,b) else False)
def dist_lt(a, b):
    return (True if a < b and not dist_eql(a,b) else False)


def time_eql(a, b):
    """Include == test to capture float('inf') == float('inf')"""
    return (True if a == b or round(abs(a-b), TIME_RND) < TIME_RES else False)
def time_ge(a, b):
    return (True if a > b or time_eql(a,b) else False)
def time_le(a, b):
    return (True if a < b or time_eql(a,b) else False)
def time_gt(a, b):
    return (True if a > b and not time_eql(a,b) else False)
def time_lt(a, b):
    return (True if a < b and not time_eql(a,b) else False)

def sec_to_hms(seconds):
    """Takes a numeric seconds value and returns a string in the format: HH:MM:SS"""
    hours = seconds//3600
    remain = seconds % 3600
    minutes = remain//60
    sec = remain % 60
    if hours:
        return "%d:%02d:%02d" % (hours, minutes, sec)
    else:
        return "%d:%02d" % (minutes, sec)