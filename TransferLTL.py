import Buchi
from Problem import problemFormulation
import datetime
import pickle
# import sys
from DetermineRoots_1st import hoftask
from DetectReuse_1st import hoftask_no_simplified, detect_reuse
from Constree import multi_trees
# from TL_RRT_star import transfer
from TransferPlanning_1st import transfer_multi_trees
from Visualization import path_plot
from Constree import tree
import numpy as np
from state import State
from sympy.logic.boolalg import to_dnf
import copy


def ftodo(todo):
    for t in todo:
        print(t[0], t[0].label, t[1], t[1].label)


def sub(subtask):
    for t in subtask.keys():
        print(t[0], t[1])


# # =========================================================================
start1 = datetime.datetime.now()

workspace, regions, centers, obs, init_state, uni_cost, formula, \
formula_comp, exclusion, num_grid = problemFormulation(0).Formulation()
ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}
#
# # # +------------------------------------------+
# # # |            construct buchi graph         |
# # # +------------------------------------------+
#
buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
buchi.formulaParser()
buchi.execLtl2ba()
_ = buchi.buchiGraph()
buchi.DelInfesEdge(len(init_state))
buchi_graph = buchi.buchi_graph

# # +------------------------------------------+
# # |        synthesize reusable skills        |
# # +------------------------------------------+
# roots for reusable skills
h_task_lib, task_group = hoftask((round(1 / num_grid / 2, 10), round(1 / num_grid / 2, 10)), buchi_graph, centers,
                                round(1 / num_grid, 10))
time1 = (datetime.datetime.now() - start1).total_seconds()
# #
# start2 = datetime.datetime.now()
# end2path = multi_trees(h_task_lib, buchi_graph, ts, centers, 40000, num_grid)
# time2 = (datetime.datetime.now() - start2).total_seconds()
#
# for key, value in end2path.items():
#     path_plot(value, regions, obs, num_grid)
#
# with open('data/lib_subtask123456_20*20_13obs_1st', 'wb+') as filehandle:
#     # store the data as binary data stream
#     pickle.dump(end2path, filehandle)
#     pickle.dump(h_task_lib, filehandle)
# # =========================================================================

# +------------------------------------------+
# |        construct new buchi graph         |
# +------------------------------------------+

with open('data/lib_subtask123456_20*20_13obs_1st', 'rb') as filehandle:
    # regions included, grid number, more obstacles, 1st subtask generation
    # store the data as binary data stream
    end2path = pickle.load(filehandle)
    h_task_lib = pickle.load(filehandle)

# new formula
workspace, regions, centers, obs, init_state, uni_cost, formula, \
formula_comp, exclusion, num_grid = problemFormulation(1).Formulation()
ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}

buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
buchi.formulaParser()
buchi.execLtl2ba()
_ = buchi.buchiGraph()
buchi.DelInfesEdge(len(init_state))
min_qb = buchi.MinLen()
buchi.FeasAcpt(min_qb)
buchi_graph = buchi.buchi_graph

# roots for new formula, including all possible states
start3 = datetime.datetime.now()
init_pos = init_state
h_task_new = hoftask_no_simplified(init_pos, buchi_graph, centers, 1 / num_grid)
time3 = (datetime.datetime.now() - start3).total_seconds()

# for x in h_task_new.nodes:
#     print(x, x.label)
# for e in h_task_new.edges:
#     print(e[0], e[1])

# +------------------------------------------+
# |         detect reusable skills           |
# +------------------------------------------+
start4 = datetime.datetime.now()
tree = tree(ts, buchi_graph, (init_pos, buchi_graph.graph['init'][0]), 0)
subtask2path, starting2waypoint, todo, todo_succ, newsubtask2subtask_p, acpt = detect_reuse(task_group, h_task_new,
                                                                                            end2path, tree)
time4 = (datetime.datetime.now() - start4).total_seconds()

# todo_succ = {(init_pos, buchi_graph.graph['init'][0]):[]}
# subtask2path = {t : [] for t in subtask2path.keys()}
# starting2waypoint = {t: set() for t in starting2waypoint.keys()}
# newsubtask2subtask_p = {t:[] for t in newsubtask2subtask_p.keys()}
print('time for graph of subtasks  : %8.5f' % time3)

print('time for reusable skills    : %8.5f' % time4)
print('number of reusable skills   : %6d' % (len(subtask2path)))
print('number of total subtasks    : %6d' % (h_task_new.number_of_edges()))
print('ratio                       : %8.2f' % (len(subtask2path) / h_task_new.number_of_edges()))
n_max = 15000
time = []
cost = []
opt_cost = np.inf
opt_path = []
# ==================================================================
for i in range(10):
    subtask2path_copy = copy.deepcopy(subtask2path)
    starting2waypoint_copy = copy.deepcopy(starting2waypoint)
    print('-------------- MultiSubtree ---- {0} time ---------------'.format(i + 1))
    start4 = datetime.datetime.now()
    p, c = transfer_multi_trees(buchi_graph, (init_pos, buchi_graph.graph['init'][0]), todo_succ, ts,
                                            centers,
                                            n_max, subtask2path_copy, starting2waypoint_copy, newsubtask2subtask_p, acpt,
                                            num_grid, dict())
    if c == np.inf:
        continue
    time4_transfer = (datetime.datetime.now() - start4).total_seconds()
    time.append(time4_transfer)
    print(time4_transfer, c)
    cost.append(c)

    if c < opt_cost:
        opt_path = p
        opt_cost = c
# ==================================================================
print('time: {0}, cost: {1}, best cost: {2}'.format(np.mean(time), np.mean(cost), opt_cost))
path_plot(opt_path, regions, obs, num_grid)
# from collections import OrderedDict
# for i in range(10):
#     start = datetime.datetime.now()
#     subtask2path_copy = copy.deepcopy(subtask2path)
#     starting2waypoint_copy = copy.deepcopy(starting2waypoint)
#     print('-------------- MultiSubtree ---- {0} time ---------------'.format(i + 1))
#     start4 = datetime.datetime.now()
#     cost_path_pre = transfer_multi_trees(buchi_graph, (init_pos, buchi_graph.graph['init'][0]), todo_succ, ts,
#                                             centers,
#                                             n_max, subtask2path_copy, starting2waypoint_copy, newsubtask2subtask_p, acpt,
#                                             num_grid, dict(), 'pre')
#     pre_time = (datetime.datetime.now() - start).total_seconds()
#     start = datetime.datetime.now()
#     opt_cost = (np.inf, np.inf)
#     opt_path_pre = []
#     opt_path_suf = []
#
#     for i in range(1):
#         # goal product state
#         goal = cost_path_pre[i][1][-1]
#         # tree_suf = tree(ts, buchi_graph, goal, goal[1], 'suf')
#
#         # label_new = bias_tree.label(goal)
#         # if 'o' in label_new:
#         #     continue
#         # if label_new != '':
#         #     label_new = label_new + '_' + str(1)
#         # if tree_suf.obs_check(tree_suf.init, tree_suf.init[0], label_new, obs_check) and tree_suf.checkTranB(
#         #         tree_suf.init[1], label_new,
#         #         tree_suf.init[1]):
#         #     opt_path_pre = cost_path_pre[i][1]  # plan of [(position, buchi)]
#         #     return pre_time, cost_path_pre[i][0], opt_path_pre
#
#         # update accepting buchi state
#         # buchi_graph.graph['accept'] = goal[1]
#         # construct suffix tree
#         cost_path_suf_cand = transfer_multi_trees(buchi_graph, goal, todo_succ, ts,
#                                     centers,
#                                     n_max, subtask2path_copy, starting2waypoint_copy, newsubtask2subtask_p, acpt,
#                                     num_grid, dict(), 'suf')
#
#         # couldn't find the path
#         try:
#             # order according to cost
#
#             cost_path_suf_cand = OrderedDict(sorted(cost_path_suf_cand.items(), key=lambda x: x[1][0]))
#             mincost = list(cost_path_suf_cand.keys())[0]
#         except IndexError:
#             del cost_path_pre[i]
#             # print('delete {0}-th item in cost_path_pre, {1} left'.format(i, len(cost_path_pre)))
#             continue
#         cost_path_suf = cost_path_suf_cand[mincost]
#
#         if cost_path_pre[i][0] + cost_path_suf[0] < opt_cost[0] + opt_cost[1]:
#             opt_path_pre = cost_path_pre[i][1]  # plan of [(position, buchi)]
#             opt_path_suf = cost_path_suf[1]
#             opt_cost = cost_path_pre[i][0] + cost_path_suf[0]  # optimal cost (pre_cost, suf_cost)
#
#         suf_time = (datetime.datetime.now() - start).total_seconds()
#         print(pre_time + suf_time, opt_cost)  # , opt_path_pre + opt_path_suf)
