import Buchi
from Problem import problemFormulation
import datetime
import pickle
# import sys
from DetermineRoots import hoftask
from DetectReuse import hoftask_no_simplified, detect_reuse
from Constree import multi_trees
# from TL_RRT_star import transfer
from TransferPlanning import transfer_multi_trees
from Visualization import path_plot
from Constree import tree

# # =========================================================================
# start1 = datetime.datetime.now()
#
# workspace, regions, centers, obs, init_state, uni_cost, formula,\
#             formula_comp, exclusion, num_grid = problemFormulation(0).Formulation()
# ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}
#
# # # +------------------------------------------+
# # # |            construct buchi graph         |
# # # +------------------------------------------+
#
# buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
# buchi.formulaParser()
# buchi.execLtl2ba()
# _ = buchi.buchiGraph()
# buchi.DelInfesEdge(len(init_state))
# buchi_graph = buchi.buchi_graph
#
# # # +------------------------------------------+
# # # |        synthesize reusable skills        |
# # # +------------------------------------------+
# # roots for reusable skills
# h_task_lib = hoftask((1/num_grid/2, 1/num_grid/2), buchi_graph, centers)
# time1 = (datetime.datetime.now() - start1).total_seconds()
#
# start2 = datetime.datetime.now()
# end2path = multi_trees(h_task_lib, buchi_graph, ts, centers, 20000, num_grid)
# time2 = (datetime.datetime.now() - start2).total_seconds()
#
# # for key, value in end2path.items():
# #     path_plot(value, regions, obs, num_grid)
#
# with open('data/lib_subtask', 'wb+') as filehandle:
#         # store the data as binary data stream
#         pickle.dump(end2path, filehandle)
#         pickle.dump(h_task_lib, filehandle)
# # =========================================================================

# +------------------------------------------+
# |        construct new buchi graph         |
# +------------------------------------------+

# new formula
start3 = datetime.datetime.now()
workspace, regions, centers, obs, init_state, uni_cost, formula, \
formula_comp, exclusion, num_grid = problemFormulation(1).Formulation()
ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}

buchi = Buchi.buchi_graph(formula, formula_comp, exclusion)
buchi.formulaParser()
buchi.execLtl2ba()
_ = buchi.buchiGraph()
buchi.DelInfesEdge(len(init_state))
buchi_graph = buchi.buchi_graph

# roots for new formula, including all possible states
init_pos = init_state
h_task_new = hoftask_no_simplified(init_pos, buchi_graph, centers)
time3 = (datetime.datetime.now() - start3).total_seconds()

# for x in h_task_new.nodes:
#     print(x, x.label)
# for e in h_task_new.edges:
#     print(e[0], e[1])

with open('data/lib_subtask123456', 'rb') as filehandle:
    # store the data as binary data stream
    end2path = pickle.load(filehandle)
    h_task_lib = pickle.load(filehandle)

# +------------------------------------------+
# |         detect reusable skills           |
# +------------------------------------------+
tree = tree(ts, buchi_graph, (init_pos, buchi_graph.graph['init'][0]), 0)
subtask2path, starting2waypoint, todo, todo_succ, newsubtask2subtask_p, acpt = detect_reuse(h_task_lib, h_task_new,
                                                                                            end2path, tree)

print('time for reusable skills  : %8.2f' % time3)
print('number of reusable skills : %6d' % (len(subtask2path)))
print('number of total subtasks  : %6d' % (h_task_new.number_of_edges()))
print('ratio                     : %8.2f' % (len(subtask2path) / h_task_new.number_of_edges()))
n_max = 15000

# ==================================================================
for i in range(10):
    print('-------------- MultiSubtree ---- {0} time ---------------'.format(i + 1))
    start4 = datetime.datetime.now()
    optpath = transfer_multi_trees(buchi_graph, (init_pos, buchi_graph.graph['init'][0]), todo_succ, ts, centers,
                                   n_max, subtask2path, starting2waypoint, newsubtask2subtask_p, acpt, num_grid, obs_check=dict())
    time4_transfer = (datetime.datetime.now() - start4).total_seconds()
    if optpath:
        p = []
        for point in optpath:
            p.append(tree.label(point))
        print(p)
        path_plot(optpath, regions, obs, num_grid)
        break
        # ==================================================================
