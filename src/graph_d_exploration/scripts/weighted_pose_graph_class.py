#!/usr/bin/python3

# jplaced@unizar.es
# 2021, Universidad de Zaragoza

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

from operator import itemgetter

debug = False


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class weighted_pose_graph:
    # Constructor
    def __init__(self, nodes=None, edges=None, criteria='d_opt'):
        self.graph = nx.Graph()
        self.criteria = criteria
        if (nodes is not None) and (edges is not None):
            for i in range(0, np.size(nodes, 0)):
                p = [nodes[i][1], nodes[i][2]]
                self.graph.add_node(nodes[i][0], pose=p, theta=nodes[i][3])
            for i in range(0, np.size(edges, 0)):
                edge = (edges[i][0], edges[i][1])
                delta = edges[i][3:6]
                I = edges[i][6:12]
                A = [[I[0], I[1], I[2]],
                     [I[1], I[3], I[4]],
                     [I[2], I[4], I[5]]]
                eigv2, _ = np.linalg.eig(A)
                eigv = eigv2[eigv2 > 1e-8]
                n = np.size(A, 1)
                if criteria == 'd_opt':
                    opt_cri = np.exp(np.sum(np.log(eigv)) / n)
                self.graph.add_edge(*edge, type=edges[i][2], delta=delta, information=I, weight=opt_cri)
        elif nodes is not None:
            print("Edges initialized to None.")
            for i in range(0, np.size(nodes, 0)):
                p = [nodes[i][1], nodes[i][2]]
                self.graph.add_node(nodes[i][0], pose=p)
        elif edges is not None:
            print("Nodes initialized to None.")
            for i in range(0, np.size(edges, 0)):
                edge = (edges[i][0], edges[i][1])
                delta = edges[i][3:6]
                I = edges[i][6:12]
                A = [[I[0], I[1], I[2]],
                     [I[1], I[3], I[4]],
                     [I[2], I[4], I[5]]]
                eigv2, _ = np.linalg.eig(A)
                eigv = eigv2[eigv2 > 1e-8]
                n = np.size(A, 1)
                if criteria == 'd_opt':
                    opt_cri = np.exp(np.sum(np.log(eigv)) / n)
                self.graph.add_edge(*edge, type=edges[i][2], delta=delta, information=I, weight=opt_cri)

    def addEdge(self, id1, id2, t, d, I):
        A = [[I[0], I[1], I[2]],
             [I[1], I[3], I[4]],
             [I[2], I[4], I[5]]]
        eigv2, _ = np.linalg.eig(A)
        eigv = eigv2[eigv2 > 1e-8]
        n = np.size(A, 1)
        if self.criteria == 'd_opt':
            opt_cri = np.exp(np.sum(np.log(eigv)) / n)
        self.graph.add_edge(id1, id2, type=t, delta=d, information=I, weight=opt_cri)

    # Creates a graph from an existing one (which may be frozen)
    def copy_graph(self, frozenG):
        self.graph.clear()
        self.graph = nx.Graph(frozenG)

    # Computes the adjacency matrix of the graph (scipy sparse matrices)
    def compute_A(self):
        return nx.adjacency_matrix(self.graph, nodelist=None, weight='weight')

    # Compute the Laplacian matrix of the graph, Laplacian = Degree - Adjacency
    def compute_L(self):
        return nx.laplacian_matrix(self.graph, weight='weight')

    # Computes the anchored Laplacian matrix along a random node
    def compute_anchored_L(self):
        L = nx.laplacian_matrix(self.graph, weight='weight')
        idx_to_drop = np.random.randint(0, np.shape(L)[1], 1)
        if debug:
            print('Laplacian anchored through index' + format(idx_to_drop))
        C = L.tocoo()
        keep = ~np.in1d(C.col, idx_to_drop)
        C.data, C.row, C.col = C.data[keep], C.row[keep], C.col[keep]
        C.col -= idx_to_drop.searchsorted(C.col)  # decrement column indices
        C._shape = (C.shape[0], C.shape[1] - len(idx_to_drop))

        keep = ~np.in1d(C.row, idx_to_drop)
        C.data, C.row, C.col = C.data[keep], C.row[keep], C.col[keep]
        C.row -= idx_to_drop.searchsorted(C.row)  # decrement column indices
        C._shape = (C.shape[0] - len(idx_to_drop), C.shape[1])

        return C.tocsr()

    # Computes all eigenvalues of the Laplacian
    def compute_spectrum(self):
        return nx.laplacian_spectrum(self.graph, weight='weight')

    # Compute the non-zero eigenvalues of the Laplacian
    def compute_reduced_spectrum(self):
        eigen_v = nx.laplacian_spectrum(self.graph, weight='weight')
        eigen_v_reduced = eigen_v[1:]
        return eigen_v_reduced

    def compute_max_eig(self):
        return np.max(nx.laplacian_spectrum(self.graph, weight='weight'))

    # Computes the algebraic connectivity of the graph
    def compute_algcon(self):
        return nx.algebraic_connectivity(self.graph, weight='weight', normalized=False, tol=1e-08, method='tracemin')

    # Computes the Fiedler vector of the graph
    def compute_fiedler(self):
        return nx.fiedler_vector(self.graph, weight='weight', normalized=False, tol=1e-08, method='tracemin')

    # Gets the edges that represent a loop closure (type 1 edges)
    # Returns an ordered list of loop closure edges (id1, id2) by its order of
    # appearance, that is, sorted by id2
    def get_LC_edges(self):
        edges = self.graph.edges.data('type')
        edges_LC = []
        for (u, v, wt) in edges:
            edges_LC.append([u, v]) if wt == 1 else True

        return sorted(edges_LC, key=itemgetter(1))

    # Gets the node number list of LC
    def get_LC_nodes_list(self):
        edges = self.graph.edges.data('type')
        idx_LC = []
        for (u, v, wt) in edges:
            if wt == 1:
                idx_LC.append([u, v])  # if u not in idx_LC[0] else True
                # idx_LC.append(v) if v not in idx_LC[1] else True
        idx_LC.sort(key=lambda x: x[1])

        return idx_LC

    # Gets the nodes where there is a LC
    def get_LC_nodes(self):
        nodes = self.graph.nodes.data('pose')
        edges = self.graph.edges.data('type')
        idx_LC = []
        for (u, v, wt) in edges:
            if wt == 1:
                idx_LC.append(u) if u not in idx_LC else True
                idx_LC.append(v) if v not in idx_LC else True
        nodes_LC = [ith_node if ith_idx == ith_node[0] else 0 for ith_idx in idx_LC for ith_node in nodes]
        nodes_LC = list(filter(lambda num: num != 0, nodes_LC))

        return nodes_LC

    # Gets the max node id
    def get_max_node_id(self):
        nodes = self.graph.nodes.data('pose')
        max_id = 0
        for (u, p) in nodes:
            if u > max_id:
                max_id = u

        return max_id

    # Gets the number of edges of the graph
    def get_no_edges(self):
        return nx.number_of_edges(self.graph)

    # Gets the number of edges of the graph
    def get_no_nodes(self):
        return nx.number_of_nodes(self.graph)

    # Draws the pose graph (i.e. trajectory)
    # Optional parameters:
    # - none, only plots the XY trajectory
    # - draw_LC_simple, also plots the starting point and the LC locations
    # - draw_LC_complex, also plots LC connections
    def plot_graph(self, label='Data', color='Blue', draw_LC_simple=False, draw_LC_complex=False):
        if self.graph is not None:
            nodes = self.graph.nodes.data('pose')
            poses_x = [el[1][0] for el in nodes]
            poses_y = [el[1][1] for el in nodes]
            plt.plot(poses_x, poses_y, '-', label=label, alpha=1, color=color)
            plt.suptitle('Trajectory')

            if draw_LC_simple and not draw_LC_complex:
                poses_LC = list(zip(*self.get_LC_nodes()))[1]
                poses_LC_x = [el[0] for el in poses_LC]
                poses_LC_y = [el[1] for el in poses_LC]
                plt.plot(poses_LC_x, poses_LC_y, '.', label='LC', color='black', alpha=1, markersize=2)
                plt.plot(poses_x[0], poses_y[0], '*', label='Start', color='black', alpha=1, markersize=10)

            elif draw_LC_complex:
                plt.plot(poses_x[0], poses_y[0], '*', label='Start', color='black', alpha=1, markersize=10)
                label_added = False
                edges_LC = self.get_LC_edges()
                for (u, v) in edges_LC:
                    poses_LC_x = []
                    poses_LC_y = []
                    poses_LC_x.append(self.graph.nodes[u]['pose'][0])
                    poses_LC_x.append(self.graph.nodes[v]['pose'][0])
                    poses_LC_y.append(self.graph.nodes[u]['pose'][1])
                    poses_LC_y.append(self.graph.nodes[v]['pose'][1])
                    if not label_added:
                        plt.plot(poses_LC_x, poses_LC_y, linestyle='-', color='orange', label='LC', alpha=1, marker='o',
                                 markeredgecolor='b', markersize=2)
                        label_added = True
                    else:
                        plt.plot(poses_LC_x, poses_LC_y, linestyle='-', color='orange', alpha=1, marker='o',
                                 markeredgecolor='b', markersize=2)
