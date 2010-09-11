"""General purpose functions, applicable across different projects"""
from __future__ import division
import math
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

def is_string_true(string):
    """Primarily intended for interpretting XML boolean input"""
    if string == 'true' or string == '1':
        return True
    else:
        return False

def is_string_false(string):
    """Primarily intended for interpretting XML boolean input"""
    if not string or string == 'false':
        return True
    else:
        return False

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

def real_roots(A, B, C, D, threshold=None):
    """Wrapper function that chooses the correct root finding function based
    on the values of the coefficients. Handles cubic polynomials and lower.
    Much, much faster than using numpy.roots and less likely to miss a tangent
    root.

    Parameters:
      A, B, C, D -- The polynomial coefficients from highest order to lowest.
      threshold -- If the inflection point of the parabola comes within
          'threshold' units of the x-axis then it will be treated as though
          it were tangent to the x-axis and a single root will be reported.
          This holds true whether the parabola didn't reach the x-axis (normally
          no real roots) or whether the parabola crossed the x-axis (normally
          two closely spaced roots).

    Results:
      A list of roots in unsorted order. May contain duplicate values if
      the polynomial is tangent to the x-axis. May be empty if there were no
      real roots.
    """
    if A != 0:
        return cubic_roots(A, B, C, D, threshold)
    elif B != 0:
        return quadratic_roots(B, C, D, threshold)
    else:
        return linear_root(C, D)

def linear_root(A, B):
    """Finds the real root of a 1st degree polynomial.

    Parameters:
      A, B -- Polynomial coefficients, from highest order to lowest.

    Returns:
      A list containing the root. List will be empty if A == 0.
    """
    try:
        return [-B/A]
    except ZeroDivisionError:
        return []

def quadratic_roots(A, B, C, threshold=None):
    """Finds the real roots of a 2nd degree polynomial, using the quadratic
    formula. Treats parabolas that come within threshold units of the x-axis
    as being tangent to it.

    Parameters:
      A, B, C -- polynomial coefficients, from highest order to lowest.
      threshold -- If the inflection point of the parabola comes within
          'threshold' units of the x-axis then it will be treated as though
          it were tangent to the x-axis and a single root will be reported.
          This holds true whether the parabola didn't reach the x-axis (normally
          no real roots) or whether the parabola crossed the x-axis (normally
          two closely spaced roots).

    Returns:
      A list of real roots.
    """
    # Find the y-value at the inflection point: -b^2/(4a) + c
    # Above eqn is the result of:
    #  1. Take derivative of generic poly: ax^2 + bx + c = 0 -> 2ax + bx = 0
    #  2. Solve for x: x = -b/(2a)
    #  3. Plug into original poly eqn: a(-b/2a)^2 + b(-b/2a) + c = 0 -> -b/(4a) + c
    try:
        B_2 = B*B
        if threshold is not None:
            y = -(B_2)/(4*A) + C
            if abs(y) <= threshold:
                C -= y # adjust the whole poly vertically so that it is tangent to x-axis

        discriminant = B_2 - 4*A*C
        if discriminant == 0: # tangent case, has one real root
            roots = [(-B/(2*A))]
        elif discriminant > 0: # crosses x-axis, has two real roots
            discriminant_root = math.sqrt(discriminant)
            q = -0.5*(B + math.copysign(discriminant_root, B)) # see Numerical Recipes Ch 5.6
            roots = [q/A, C/q]
        else: # discriminant < 0; doesn't cross x-axis, only imaginary roots
            roots = []
        return roots
    except ZeroDivisionError:
        raise ValueError("Not a quadratic polynomial")

def cubic_roots(A,B,C,D, threshold=None):
    """Finds the real roots of a third degree polynomial.

    Parameters:
      A,B,C,D -- polynomial coefficients, from highest order to lowest.

    Returns:
      A list of real roots.
    """
    if A == 0:
        raise ValueError("Not a cubic polynomial")

    middle_root_only = False
    if threshold is not None:
        # Find the y-values at the two inflection points
        # Values at inflection point found by using Mathematica with the following code:
        # f[x_] := x^3 + a*x^2 + b*x + c
        # inflectPts = Solve[f'[x] == 0, x]
        # f[x] /. inflectPts
        try:
            neg = (-B - math.sqrt(B*B - 3*A*C))/(3*A)
            neg_2 = neg*neg
            neg_3 = neg_2*neg
            y1 = A*neg_3 + B*neg_2 + C*neg + D

            pos = (-B + math.sqrt(B*B - 3*A*C))/(3*A)
            pos_2 = pos*pos
            pos_3 = pos_2*pos
            y2 = A*pos_3 + B*pos_2 + C*pos + D

            if abs(y1) > threshold and abs(y2) > threshold:
                pass
            elif abs(y1) <= threshold and abs(y2) > threshold:
                D -= y1 # correct the spline vertically so that it's just tangent with x-axis at inflection 1
            elif abs(y1) > threshold and abs(y2) <= threshold:
                D -= y2 # tangent at inflection 2
            elif abs(y1) <= threshold and abs(y2) <= threshold:
                # Cubic crosses the x-axis three times without exceeding threshold.
                # Collapse the three real roots to just the middle root. Adjust the
                # poly so that all three roots will be real, then use just the middle one.
                middle_root_only = True
                ave = (y1 + y2)/2
                D -= ave
            else:
                raise Exception("Unexpected case.")

        except ValueError:
            assert B*B - 3*A*C < 0 # only one inflection point anyways(?)
            pass

    # Use the form: x^3 + ax^2 + bx + c = 0
    a = B/A
    b = C/A
    c = D/A

    a_2 = a*a
    a_3 = a_2*a

    Q = (a_2 - 3*b)/9
    R = (2*a_3 - 9*a*b + 27*c)/54

    R_2 = R*R
    Q_3 = Q*Q*Q

    # Allow for some rounding error, with an emphasis on finding more roots
    if abs(R_2 - Q_3) < 1E-14:
        if R_2 > 0:
            Q_3 = R_2
            Q = math.pow(Q_3, 1/3)

    if R_2 <= Q_3 and Q_3 != 0: # Has three real roots
        RQ_temp = R/math.pow(Q, 3/2)
        try:
            sigma = math.acos(RQ_temp)
        except ValueError:
            if RQ_temp > 1:
                assert RQ_temp < 1.01
                sigma = 0.0 # acos(1)
            elif RQ_temp < -1:
                assert RQ_temp > -1.01
                sigma = math.pi # acos(-1)
            else:
                raise
        sq_root_Q = math.sqrt(Q)
        roots = [-2 * sq_root_Q * math.cos(sigma/3) - a/3,
                 -2 * sq_root_Q * math.cos((sigma + 2*math.pi)/3) - a/3,
                 -2 * sq_root_Q * math.cos((sigma - 2*math.pi)/3) - a/3]

    else: # Just one real root
        S = -math.copysign(1, R)*math.pow(abs(R) + math.sqrt(R_2 - Q_3), 1/3)
        T = 0 if S == 0 else Q/S
        roots = [(S + T) - a/3]

    if middle_root_only and len(roots) == 3:
        roots = [roots[1]]

    return roots

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
    # Include == test to capture float('inf') == float('inf')
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

def latlng_dist(lat1, lng1, lat2, lng2):
    """Returns the great circle distance between two coordinates."""
    return 2*math.asin(math.sqrt((math.sin((lat1-lat2)/2))^2 + \
                            math.cos(lat1)*math.cos(lat2)*(math.sin((lng1-lng2)/2))^2))
