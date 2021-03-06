# -*- coding: utf-8 -*-
"""
Maximum flow (and minimum cut) algorithms on capacitated graphs.
"""

__author__ = u"""Loïc Séguin-C. <loicseguin@gmail.com>"""
u"""Copyright (C) 2010 Loïc Séguin-C. <loicseguin@gmail.com>
All rights reserved.
BSD license.
"""

__all__ = ['ford_fulkerson',
           'max_flow',
           'min_cut']

import networkx as nx

def _create_auxiliary_digraph(G):
    """Initialize an auxiliary digraph for a given graph G.
    Ignore edges with capacity <= 0.
    """
    auxiliary = nx.DiGraph()

    if nx.is_directed(G):
        for edge in G.edges(data = True):
            if edge[2].has_key('capacity'):
                if edge[2]['capacity'] > 0:
                    auxiliary.add_edge(*edge)
            else:
                auxiliary.add_edge(*edge)
    else:
        for edge in G.edges(data = True):
            if edge[2].has_key('capacity'):
                if edge[2]['capacity'] > 0:
                    auxiliary.add_edge(*edge)
                    auxiliary.add_edge(edge[1], edge[0], edge[2])
            else:
                auxiliary.add_edge(*edge)
                auxiliary.add_edge(edge[1], edge[0], edge[2])

    return auxiliary


def _create_flow_graph(G, H):
    """Creates the flow graph on G corresponding to the auxiliary
    digraph H.
    """
    if nx.is_directed(G):
        flowGraph = nx.DiGraph(G)
    else:
        flowGraph = nx.Graph(G)

    for (u, v) in flowGraph.edges():
        if H.has_edge(u, v):
            try:
                flowGraph[u][v]['flow'] = abs(G[u][v]['capacity']
                                              - H[u][v]['capacity'])
            except KeyError:
                try:
                    flowGraph[u][v]['flow'] = H[v][u]['capacity']
                except KeyError:
                    # If we end up here, it is because of an infinite
                    # capacity digon in the original graph. A flow
                    # value could be computed using flow conservation.
                    # For now, we just set the flow value to None.
                    flowGraph[u][v]['flow'] = None
        else:
            flowGraph[u][v]['flow'] = G[u][v]['capacity']

    return flowGraph


def ford_fulkerson(G, s, t):
    """Find a maximum single-commodity flow using the Ford-Fulkerson
    algorithm.
    
    This algorithm uses Edmond-Karp-Dinitz path selection rule which
    guarantees a running time of O(nm**2) where n is the number of
    nodes and m is the number of edges.

    Parameters
    ----------
    G : NetworkX graph
        Edges of the graph are expected to have an attribute called
        'capacity'. If this attribute is not present, the edge is
        considered to have infinite capacity.

    s : node
        Source node for the flow.

    t : node
        Sink node for the flow.

    Returns
    -------
    flowValue : integer, float
        Value of the maximum flow, i.e., net outflow from the source.

    flowGraph : NetworkX graph
        Graph with V(flowGraph) = V(G) and in which each edge has an
        attribute 'flow' which gives the flow on the edge.

    Examples
    --------
    >>> import networkx as nx
    >>> import maxflow
    >>> G = nx.DiGraph()
    >>> G.add_edge('x','a', capacity = 3.0)
    >>> G.add_edge('x','b', capacity = 1.0)
    >>> G.add_edge('a','c', capacity = 3.0)
    >>> G.add_edge('b','c', capacity = 5.0)
    >>> G.add_edge('b','d', capacity = 4.0)
    >>> G.add_edge('d','e', capacity = 2.0)
    >>> G.add_edge('c','y', capacity = 2.0)
    >>> G.add_edge('e','y', capacity = 3.0)
    >>> flow,F=maxflow.ford_fulkerson(G, 'x', 'y')
    >>> flow
    3.0
    """
    
    auxiliary = _create_auxiliary_digraph(G)
    flowValue = 0   # Initial feasible flow.

    # As long as there is an (s, t)-path in the auxiliary digraph, find
    # the shortest (with respect to the number of arcs) such path and
    # augment the flow on this path.
    while True:
        pathNodes = nx.bidirectional_shortest_path(auxiliary, s, t)
        if not pathNodes:
            break

        # Get the list of edges in the shortest path.
        pathEdges = []
        for i, u in enumerate(pathNodes[:-1]):
            v = pathNodes[i + 1]
            pathEdges.append((u, v, auxiliary[u][v]))

        # Find the minimum capacity of an edge in the path.
        try:
            pathCapacity = min([c['capacity']
                            for (u, v, c) in pathEdges
                            if c.has_key('capacity')])
        except ValueError: 
            return None # path of infinite capacity implies no max flow
        
        flowValue += pathCapacity

        # Augment the flow along the path.
        for (u, v, c) in pathEdges:
            auxEdgeAttr = auxiliary[u][v]
            if auxEdgeAttr.has_key('capacity'):
                auxEdgeAttr['capacity'] -= pathCapacity
                if auxEdgeAttr['capacity'] == 0:
                    auxiliary.remove_edge(u, v)

            if auxiliary.has_edge(v, u):
                if auxiliary[v][u].has_key('capacity'):
                    auxiliary[v][u]['capacity'] += pathCapacity
            else:
                auxiliary.add_edge(v, u, {'capacity': pathCapacity})
    
    flowGraph = _create_flow_graph(G, auxiliary)
    return flowValue, flowGraph


# Just an alias for max_flow
max_flow = ford_fulkerson


def min_cut(G, s, t):
    """Compute the value of a minimum (s, t)-cut.

    Use the max-flow min-cut theorem, i.e., the capacity of a minimum
    capacity cut is equal to the flow value of a maximum flow.

    Parameters
    ----------
    G : NetworkX graph
        Edges of the graph are expected to have an attribute called
        'capacity'. If this attribute is not present, the edge is
        considered to have infinite capacity.

    s : node
        Source node for the flow.

    t : node
        Sink node for the flow.

    Returns
    -------
    cutValue : integer, float
        Value of the minimum cut.

    Examples
    --------
    >>> import networkx as nx
    >>> import maxflow
    >>> G = nx.DiGraph()
    >>> G.add_edge('x','a', capacity = 3.0)
    >>> G.add_edge('x','b', capacity = 1.0)
    >>> G.add_edge('a','c', capacity = 3.0)
    >>> G.add_edge('b','c', capacity = 5.0)
    >>> G.add_edge('b','d', capacity = 4.0)
    >>> G.add_edge('d','e', capacity = 2.0)
    >>> G.add_edge('c','y', capacity = 2.0)
    >>> G.add_edge('e','y', capacity = 3.0)
    >>> maxflow.min_cut(G, 'x', 'y')
    3.0
    """

    try:
        return ford_fulkerson(G, s, t)[0]
    except TypeError:
        return None

