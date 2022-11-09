#!/usr/bin/python
#
# Julio Placed. University of Zaragoza. 2022.
# jplaced@unizar.es

from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
import sys
import time
import warnings
from optparse import OptionParser
from weighted_pose_graph_class import weighted_pose_graph
import utils as ut
from pandas import DataFrame
python_version = sys.version_info[0]
if python_version < 3:
    warnings.warn("Careful, using python version 2")

if __name__ == '__main__':
    try:
        fig_id = 0
        parser = OptionParser()
        parser.add_option("--graph", dest="graph_name", default="FRH_P_toro")
        parser.add_option("--initial_nodes", dest="initial_nodes", default="FRH_P_toro_nodes.txt")
        parser.add_option("--initial_edges", dest="initial_edges", default="FRH_P_toro_edges.txt")
        parser.add_option("--optimized_nodes", dest="optimized_nodes", default="FRH_P_toro_opt_nodes.txt")
        parser.add_option("--optimized_edges", dest="optimized_edges", default="FRH_P_toro_opt_edges.txt")
        (options, args) = parser.parse_args()
        [nodes_i, edges_i, nodes_o, edges_o] = ut.read_graph(options, args)
        G_i = weighted_pose_graph(nodes_i, edges_i)
        G_o = weighted_pose_graph(nodes_o, edges_o)
        G_t = weighted_pose_graph(nodes_i, edges_i, 't_opt')
        G_d = weighted_pose_graph(nodes_i, edges_i, 'd_opt')
        G_e = weighted_pose_graph(nodes_i, edges_i, 'e_opt')
        G_te = weighted_pose_graph(nodes_i, edges_i, 'tilde_opt')

        print('Pose graph plot.')
        ut.wait_enterKey()
        fig_id += 1
        plt.figure(fig_id)
        ax = plt.gca()
        G_i.plot_graph('Trajectory', 'blue', False)
        G_o.plot_graph('Trajectory', 'red', True)
        plt.suptitle('Pose graph trajectory', fontsize=16)
        plt.xlabel('X (m)', fontsize=12)
        plt.ylabel('Y (m)', fontsize=12)
        plt.axis('equal')
        plt.legend()
        plt.grid()
        plt.show(block=1)

        print('Computation of spectral properties of the full graph.')
        ut.wait_enterKey()
        avg = 2 * G_i.get_no_edges() / G_i.get_no_nodes()
        eigv_2 = G_i.compute_algcon()
        L_anch = G_i.compute_anchored_L()
        sign, logdet = np.linalg.slogdet(L_anch.todense())
        if sign == 1:
            spann = logdet / ((G_i.get_no_nodes() - 2) * np.log(G_i.get_no_nodes()))
        else:
            print("WARNING. Check slogdet, signed below or equal to zero.")
            spann = 0
        print('Average Degree (d): ' + format(avg))
        print('Algebraic Connectivity (lambda_2): ' + format(eigv_2))
        print('Normalized Tree connectivity = log(t)/log(n)*n-2: ' + format(spann))
        FIM = ut.build_fullFIM(G_i)
        fig, ax = plt.subplots()
        ax.spy(FIM, markersize=1, color='black')
        plt.show(block=1)

        print('Sequential analysis of the full graph.')
        ut.wait_enterKey()
        reduced_G = weighted_pose_graph()
        reduced_G_t = weighted_pose_graph()
        reduced_G_d = weighted_pose_graph()
        reduced_G_e = weighted_pose_graph()
        reduced_G_te = weighted_pose_graph()
        nodes_idx = G_i.graph.nodes
        id1_pre = 0
        id2_pre = 0
        opt_criteria2 = []
        opt_criteria3 = []
        opt_criteria4 = []
        opt_criteria_max = []
        graph_measure_t = []
        graph_measure_deg = []
        graph_measure_alg = []
        graph_measure_max = []
        total_time = 0
        timing_L = []
        timing_FIM = []

        for idx in range(0, 300):
        # for idx in nodes_idx:
            if idx > 0:
                subgraph_nodes = range(0, int(idx) + 1)
                if idx % 10 == 0:
                    print("Node: " + format(idx))

                tic = time.time()
                reduced_G.graph = G_i.graph.subgraph(subgraph_nodes)
                reduced_G_t.graph = G_t.graph.subgraph(subgraph_nodes)
                reduced_G_d.graph = G_d.graph.subgraph(subgraph_nodes)
                reduced_G_e.graph = G_e.graph.subgraph(subgraph_nodes)
                reduced_G_te.graph = G_te.graph.subgraph(subgraph_nodes)
                n_nodes = idx + 1
                n_edges = reduced_G_d.get_no_edges()
                L_anch = reduced_G_d.compute_anchored_L()
                _, t = np.linalg.slogdet(L_anch.todense())
                metric_spanning = n_nodes ** (1 / n_nodes) * np.exp(t / n_nodes)
                graph_measure_t.append(metric_spanning)
                metric_mu2 = reduced_G_e.compute_algcon()
                graph_measure_alg.append(metric_mu2)
                degs = [x[1] for x in reduced_G_t.graph.degree(weight='weight')]
                metric_degs = np.mean(degs)
                graph_measure_deg.append(metric_degs)
                metric_max = reduced_G_te.compute_maxE()
                graph_measure_max.append(metric_max)
                toc = (time.time() - tic)
                timing_L.append(toc)

                tic = time.time()
                FIM = ut.build_fullFIM(reduced_G)
                t_opt_FIM, d_opt_FIM, a_opt_FIM, e_opt_FIM, tilde_opt_FIM = ut.compute_optimality(FIM, e_opt='both', invert_matrix=False)
                if not np.isnan(e_opt_FIM) and not np.isinf(e_opt_FIM):
                    opt_criteria2.append(t_opt_FIM)
                    opt_criteria3.append(d_opt_FIM)
                    opt_criteria4.append(e_opt_FIM)
                    opt_criteria_max.append(tilde_opt_FIM)
                else:
                    warnings.warn("!!! Inf/NaN due to <0 eigenvalues in covariance matrix. Check node: " + format(idx))
                    ut.wait_enterKey()
                toc = (time.time() - tic)
                timing_FIM.append(toc)

        fig_id += 1
        h = plt.figure(fig_id)
        ax5 = h.add_subplot(412)
        ax6 = h.add_subplot(413)
        ax7 = h.add_subplot(414)
        ax8 = h.add_subplot(411)
        x_series = range(0, len(opt_criteria2))
        ax8.plot(x_series, opt_criteria_max, 'b', label=r'$\tilde{E}$-opt(FIM)')
        ax8.plot(x_series, graph_measure_max, 'r', label=r'$\tilde{E}$-opt($L_w$)')
        ax8.grid()
        ax5.plot(x_series, opt_criteria2, 'b', label=r'T-opt(FIM)')
        ax5.plot(x_series, graph_measure_deg, 'r', label=r'T-opt($L_w$)')
        ax5.grid()
        ax6.plot(x_series, opt_criteria3, 'b', label=r'D-opt(FIM)')
        ax6.plot(x_series, graph_measure_t, 'r', label=r'D-opt($L_w$)')
        ax6.grid()
        ax7.plot(x_series, opt_criteria4, 'b', label=r'E-opt(FIM)')
        ax7.plot(x_series, graph_measure_alg, 'r', label=r'E-opt($L_w$)')
        ax7.grid()
        ax5.legend(loc='best')
        ax6.legend(loc='best')
        ax7.legend(loc='best')
        ax8.legend(loc='best')
        h.text(0.5, 0.04, r'No. of nodes', ha='center', va='center')
        h.text(0.04, 0.5, r'Optimality criteria', ha='center', va='center', rotation='vertical')
        fig_id += 1
        f = plt.figure(fig_id)
        plt.xlabel('No. of nodes')
        plt.ylabel('t(s)')
        ax = plt.gca()
        timing_L = DataFrame(timing_L)
        timing_FIM = DataFrame(timing_FIM)
        ax.plot(timing_L.index, timing_L.values * 1, 'salmon')
        ax.plot(timing_L.index, timing_L.rolling(10).mean().values * 1, 'r', label=r'$\|L_w\|_p$')
        ax.plot(timing_FIM.index, timing_FIM.values, 'cornflowerblue')
        ax.plot(timing_FIM.index, timing_FIM.rolling(10).mean().values, 'b', label=r'$\|Y\|_p$')
        plt.grid(True, which='both')
        plt.minorticks_on()
        ax.legend(loc='upper left')
        plt.axis([None, None, None, None])
        plt.show(block=1)

    except KeyboardInterrupt:
        plt.close('all')
        print('\n Exit requested.')
        sys.exit()
