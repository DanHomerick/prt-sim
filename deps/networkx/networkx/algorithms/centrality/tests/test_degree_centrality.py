"""
    Unit tests for degree centrality.
"""

from nose.tools import *

import networkx as nx


class TestDegreeCentrality:
    def __init__(self):

        self.K = nx.krackhardt_kite_graph()
        self.P3 = nx.path_graph(3)
        self.K5 = nx.complete_graph(5)

        F = nx.Graph() # Florentine families
        F.add_edge('Acciaiuoli','Medici')
        F.add_edge('Castellani','Peruzzi')
        F.add_edge('Castellani','Strozzi')
        F.add_edge('Castellani','Barbadori')
        F.add_edge('Medici','Barbadori')
        F.add_edge('Medici','Ridolfi')
        F.add_edge('Medici','Tornabuoni')
        F.add_edge('Medici','Albizzi')
        F.add_edge('Medici','Salviati')
        F.add_edge('Salviati','Pazzi')
        F.add_edge('Peruzzi','Strozzi')
        F.add_edge('Peruzzi','Bischeri')
        F.add_edge('Strozzi','Ridolfi')
        F.add_edge('Strozzi','Bischeri')
        F.add_edge('Ridolfi','Tornabuoni')
        F.add_edge('Tornabuoni','Guadagni')
        F.add_edge('Albizzi','Ginori')
        F.add_edge('Albizzi','Guadagni')
        F.add_edge('Bischeri','Guadagni')
        F.add_edge('Guadagni','Lamberteschi')    
        self.F = F

        G = nx.DiGraph()
        G.add_edge(0,5)
        G.add_edge(1,5)
        G.add_edge(2,5)
        G.add_edge(3,5)
        G.add_edge(4,5)
        G.add_edge(5,6)
        G.add_edge(5,7)
        G.add_edge(5,8)
        self.G = G

    def test_degree_centrality_1(self):
        d = nx.degree_centrality(self.K5)
        exact = dict(zip(range(5), [1]*5))
        for n,dc in d.iteritems():
            assert_almost_equal(exact[n], dc)

    def test_degree_centrality_2(self):
        d = nx.degree_centrality(self.P3)
        exact = {0:0.5, 1:1, 2:0.5}
        for n,dc in d.iteritems():
            assert_almost_equal(exact[n], dc)

    def test_degree_centrality_3(self):
        d = nx.degree_centrality(self.K)
        exact = {0:.444, 1:.444, 2:.333, 3:.667, 4:.333,
                 5:.556, 6:.556, 7:.333, 8:.222, 9:.111}
        for n,dc in d.iteritems():
            assert_almost_equal(exact[n], float("%5.3f" % dc))

    def test_degree_centrality_4(self):
        d = nx.degree_centrality(self.F)
        names = sorted(self.F.nodes())
        dcs = [0.071, 0.214, 0.143, 0.214, 0.214, 0.071, 0.286, 
               0.071, 0.429, 0.071, 0.214, 0.214, 0.143, 0.286, 0.214]
        exact = dict(zip(names, dcs))
        for n,dc in d.iteritems():
            assert_almost_equal(exact[n], float("%5.3f" % dc))

    def test_indegree_centrality(self):
        d = nx.in_degree_centrality(self.G)
        exact = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 
                 5: 0.625, 6: 0.125, 7: 0.125, 8: 0.125}
        for n,dc in d.iteritems():
            assert_almost_equal(exact[n], dc)

    def test_outdegree_centrality(self):
        d = nx.out_degree_centrality(self.G)
        exact = {0: 0.125, 1: 0.125, 2: 0.125, 3: 0.125,    
                 4: 0.125, 5: 0.375, 6: 0.0, 7: 0.0, 8: 0.0}
        for n,dc in d.iteritems():
            assert_almost_equal(exact[n], dc)

