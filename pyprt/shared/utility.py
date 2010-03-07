"""General purpose functions, applicable across different projects"""

import itertools

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
        return "%2d:%02d:%02d" % (hours, minutes, sec)
    else:
        return "%2d:%02d" % (minutes, sec)