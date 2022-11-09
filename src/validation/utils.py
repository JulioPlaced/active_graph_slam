#!/usr/bin/python
#
# Julio Placed. University of Zaragoza. 2022.
# jplaced@unizar.es

import networkx as nx
import numpy as np
import heapq
import scipy
import sys
python_version = sys.version_info
from constants import NMINEIG, EIG_TH

def wait_enterKey():
    input("Press Enter to continue...") if (python_version >= (3, 0)) else raw_input("Press Enter to continue...")

def enforce_symmetry_list(A):
    A = np.array(A)
    return np.tril(A.T) + np.triu(A, 1)

def read_graph(options, args):
    nodes_o = None;
    edges_o = None;
    edges_oo = None;

    if options.graph_name != 'FRH_P_toro':
        print('Not default graph: ' + format(options.graph_name))
        if options.graph_name[-3:] == 'g2o':
            options.initial_nodes = options.graph_name + "_nodes.txt"
            options.initial_edges = options.graph_name + "_edges.txt"
            options.optimized_nodes = options.graph_name + "_opt_nodes.txt"
            options.optimized_edges = options.graph_name + "_opt_edges.txt"
        elif options.graph_name[-4:] == 'toro':
            options.initial_nodes = options.graph_name + "_nodes.txt"
            options.initial_edges = options.graph_name + "_edges.txt"
            options.optimized_nodes = options.graph_name + "_opt_nodes.txt"
            options.optimized_edges = options.graph_name + "_opt_edges.txt"
    else:
        print('Default graph.')

    if options.initial_nodes != '':
        nodes_o = np.genfromtxt(options.initial_nodes, usecols=(0, 1, 2, 3))
    if options.initial_edges != '':
        edges_o = np.genfromtxt(options.initial_edges, usecols=(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11))
    if options.optimized_nodes != '':
        nodes_opt = np.genfromtxt(options.optimized_nodes, usecols=(0, 1, 2, 3))
    if options.optimized_edges != '':
        edges_oo = np.genfromtxt(options.optimized_edges, usecols=(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11))

    return nodes_o, edges_o, nodes_opt, edges_oo

def compute_optimality(A, e_opt='max', invert_matrix=False):
    if invert_matrix: A = np.linalg.pinv(A)

    eigv2 = scipy.linalg.eigvalsh(A)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")

    eigv = eigv2[eigv2 > EIG_TH]
    n = np.size(A, 1)
    t_opt = np.sum(eigv) / n
    d_opt = np.exp(np.sum(np.log(eigv)) / n)
    a_opt = n / np.sum(1. / eigv)

    if e_opt == 'min':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        return t_opt, d_opt, a_opt, e_opt
    elif e_opt == 'max':
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, tilde_opt
    elif e_opt == 'both':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, e_opt, tilde_opt

def compute_optimality_sparse(A, e_opt='max', invert_matrix=False):
    if invert_matrix: A = np.linalg.pinv(A)

    eigv2 = scipy.sparse.linalg.eigsh(A, k= min(len(A.todense())-1,6), return_eigenvectors=False)
    if np.iscomplex(eigv2.any()):
        print("Error: Complex Root")

    eigv = eigv2[eigv2 > EIG_TH]
    n = np.size(A, 1)
    t_opt = np.sum(eigv) / n
    d_opt = np.exp(np.sum(np.log(eigv)) / n)
    a_opt = n / np.sum(1. / eigv)

    if e_opt == 'min':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        return t_opt, d_opt, a_opt, e_opt
    elif e_opt == 'max':
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, tilde_opt
    elif e_opt == 'both':
        e_opt = heapq.nsmallest(NMINEIG, eigv)[-1]
        tilde_opt = np.max(eigv)
        return t_opt, d_opt, a_opt, e_opt, tilde_opt

def build_fullFIM(graph):
    graph_size = nx.number_of_nodes(graph.graph)
    dim = 3
    A = np.zeros((graph_size * dim, graph_size * dim))

    for i in range(0, graph_size):
        edge_Info = graph.graph.edges([i], 'information')
        for (id1, id2, fisher) in edge_Info:
            node1 = int(id1);
            node2 = int(id2)
            if node2 > node1:
                FIM = fisher
                FIM = np.array(FIM)
                A[(node2) * dim:(node2 + 1) * dim, (node2) * dim:(node2 + 1) * dim] += FIM
                A[(node1) * dim:(node1 + 1) * dim, (node1) * dim:(node1 + 1) * dim] += FIM
                A[(node1) * dim:(node1 + 1) * dim, (node2) * dim:(node2 + 1) * dim] = - FIM
                A[(node2) * dim:(node2 + 1) * dim, (node1) * dim:(node1 + 1) * dim] = - FIM

    diff = A - A.T
    if not np.all(np.abs(diff.data) < 1e-8):
        print("Error in build_fullFIM: Fisher Information matrix is not symmetric.")

    return A
