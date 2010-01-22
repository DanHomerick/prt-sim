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