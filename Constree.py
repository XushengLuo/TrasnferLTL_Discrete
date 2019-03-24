"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 8/30/18
"""
"""
construct trees for biased sampling optimal task planning for multi-robots
"""

from random import uniform
from networkx.classes.digraph import DiGraph
from networkx.algorithms import dfs_labeled_edges
import math
import numpy as np
from collections import OrderedDict
from shapely.geometry import Point, Polygon, LineString
from datetime import datetime


class tree(object):
    """ construction of prefix and suffix tree
    """
    def __init__(self, ts, buchi_graph, init, base=1e3, seg='pre'):
        """
        :param ts: transition system
        :param buchi_graph:  Buchi graph
        :param init: product initial state
        """
        self.robot = 1
        self.goals = []
        self.ts = ts
        self.buchi_graph = buchi_graph
        self.init = init
        self.dim = len(self.ts['workspace'])
        self.tree = DiGraph(type='PBA', init=init)
        label = self.label(init[0])
        if label != '':
            label = label + '_' + str(1)
        # accepting state before current node
        acc = set()
        if 'accept' in init[1]:
            acc.add(init)
        self.tree.add_node(init, cost=0, label=label, acc=acc)

        # already used skilles
        self.used = set()
        self.base = base
        self.seg = seg
        self.found = 10

    def sample(self, num):
        """
        sample point from the workspace
        :return: sampled point, tuple
        """
        q_rand = list(self.tree.nodes())[np.random.randint(self.tree.number_of_nodes(), size=1)[0]]
        x_rand = [0, 0]
        # parallel to one axis
        line = np.random.randint(2, size=1)[0]
        x_rand[line] = q_rand[0][line]
        # sample another component
        r = round(1/num,10)
        # x_rand[1-line] = int(np.random.uniform(0, 1, size=1)[0]/r) * r + r/2
        x_rand[1-line] = round(np.random.randint(num, size=1)[0] * r + r/2, 10)
        return tuple(x_rand)

    def acpt_check(self, q_min, q_new):
        """
        check the accepting state in the patg leading to q_new
        :param q_min:
        :param q_new:
        :return:
        """
        changed = False
        acc = set(self.tree.nodes[q_min]['acc'])  # copy
        if 'accept' in q_new[1]:
            acc.add(q_new)
            # print(acc)
            changed = True
        return acc, changed

    def extend(self, q_new, prec_list, succ_list, label_new):
        """
        :param: q_new: new state form: tuple (mulp, buchi)
        :param: near_v: near state form: tuple (mulp, buchi)
        :param: obs_check: check obstacle free  form: dict { (mulp, mulp): True }
        :param: succ: list of successor of the root
        :return: extending the tree
        """

        added = 0
        cost = np.inf
        q_min = ()
        for pre in prec_list:
            if pre in succ_list:  # do not extend if there is a corresponding root
                continue
            c = self.tree.nodes[pre]['cost'] + np.abs(q_new[0][0]-pre[0][0]) + np.abs(q_new[0][1]-pre[0][1])
            if c < cost:
                added = 1
                q_min = pre
                cost = c
        if added == 1:
            self.tree.add_node(q_new, cost=cost, label=label_new)
            self.tree.nodes[q_new]['acc'] = set(self.acpt_check(q_min, q_new)[0])
            self.tree.add_edge(q_min, q_new)
        return added

    def rewire(self, q_new, succ_list):
        """
        :param: q_new: new state form: tuple (mul, buchi)
        :param: near_v: near state form: tuple (mul, buchi)
        :param: obs_check: check obstacle free form: dict { (mulp, mulp): True }
        :return: rewiring the tree
        """
        for suc in succ_list:
            # root
            if suc == self.init:
                continue
            c = self.tree.nodes[q_new]['cost'] + np.abs(q_new[0][0]-suc[0][0]) + np.abs(q_new[0][1]-suc[0][1])
            delta_c = self.tree.nodes[suc]['cost'] - c
            # update the cost of node in the subtree rooted at near_vertex
            if delta_c > 0:
                self.tree.remove_edge(list(self.tree.pred[suc].keys())[0], suc)
                self.tree.add_edge(q_new, suc)
                edges = dfs_labeled_edges(self.tree, source=suc)
                acc, changed = self.acpt_check(q_new, suc)
                self.tree.nodes[suc]['acc'] = set(acc)
                for u, v, d in edges:
                    if d == 'forward':
                        self.tree.nodes[v]['cost'] = self.tree.nodes[v]['cost'] - delta_c
                        if changed:
                            self.tree.nodes[v]['acc'] = set(self.acpt_check(u, v)[0])  # copy
        # better to research the goal but abandon the implementation

    def prec(self, q_new, label_new, obs_check):
        """
        find the predcessor of q_new
        :param q_new: new product state
        :return: label_new: label of new
        """
        p_prec = []
        for vertex in self.tree.nodes:
            if q_new != vertex and self.obs_check(vertex, q_new[0], label_new, obs_check) \
                    and self.checkTranB(vertex[1], self.tree.nodes[vertex]['label'], q_new[1]):
                p_prec.append(vertex)
        return p_prec

    def succ(self, q_new, label_new, obs_check):
        """
        find the successor of q_new
        :param q_new: new product state
        :return: label_new: label of new
        """
        p_succ = []
        for vertex in self.tree.nodes:
            if q_new != vertex and self.obs_check(vertex, q_new[0], label_new, obs_check) \
                    and self.checkTranB(q_new[1], self.tree.nodes[q_new]['label'], vertex[1]):
                p_succ.append(vertex)
        return p_succ

    def obs_check(self, q_tree, x_new, label_new, obs_check):
        """
        check whether obstacle free along the line from q_tree to x_new
        :param q_tree: vertex in the tree
        :param x_new:
        :param label_new:
        :return: true or false
        """

        if q_tree[0][0] != x_new[0] and q_tree[0][1] != x_new[1]:
            return False

        if (q_tree[0], x_new) in obs_check.keys():
            return obs_check[(q_tree[0], x_new)]

        if (x_new, q_tree[0]) in obs_check.keys():
            return obs_check[(x_new, q_tree[0])]

        # the line connecting two points crosses an obstacle
        for (obs, boundary) in iter(self.ts['obs'].items()):
            if LineString([Point(q_tree[0]), Point(x_new)]).intersects(boundary):
                obs_check[(q_tree[0], x_new)] = False
                obs_check[(x_new, q_tree[0])] = False
                return False

        for (region, boundary) in iter(self.ts['region'].items()):
                if LineString([Point(q_tree[0]), Point(x_new)]).intersects(boundary) \
                        and region + '_' + str(1) != label_new \
                        and region + '_' + str(1) != self.tree.nodes[q_tree]['label']:
                    obs_check[(q_tree[0], x_new)] = False
                    obs_check[(x_new, q_tree[0])] = False
                    return False

        obs_check[(q_tree[0], x_new)] = True
        obs_check[(x_new, q_tree[0])] = True
        return True

    def label(self, x):
        """
        generating the label of position state
        :param x: position
        :return: label
        """

        point = Point(x)
        # whether x lies within obstacle
        for (obs, boundary) in iter(self.ts['obs'].items()):
            if point.within(boundary):
                return obs

        # whether x lies within regions
        for (region, boundary) in iter(self.ts['region'].items()):
            if point.within(boundary):
                return region
        # x lies within unlabeled region
        return ''

    def checkTranB(self, b_state, x_label, q_b_new):
        """ decide valid transition, whether b_state --L(x)---> q_b_new
             :param b_state: buchi state
             :param x_label: label of x
             :param q_b_new buchi state
             :return True satisfied
        """
        b_state_succ = self.buchi_graph.succ[b_state]
        # q_b_new is not the successor of b_state
        if q_b_new not in b_state_succ:
            return False

        truth = self.buchi_graph.edges[(b_state, q_b_new)]['truth']
        if self.t_satisfy_b_truth(x_label, truth):
            return True

        return False

    def t_satisfy_b_truth(self, x_label, truth):
        """
        check whether transition enabled under current label
        :param x_label: current label
        :param truth: truth value making transition enabled
        :return: true or false
        """
        if truth == '1':
            return True

        true_label = [truelabel for truelabel in truth.keys() if truth[truelabel]]
        for label in true_label:
            if label not in x_label:
                return False

        false_label = [falselabel for falselabel in truth.keys() if not truth[falselabel]]
        for label in false_label:
            if label in x_label:
                return False

        return True

    def findpath(self, goals):
        """
        find the path backwards
        :param goals: goal state
        :return: dict path : cost
        """
        paths = OrderedDict()
        for i in range(len(goals)):
            goal = goals[i]
            path = [goal]
            s = goal
            while s != self.init:
                s = list(self.tree.pred[s].keys())[0]
                path.insert(0, s)
            paths[i] = [self.tree.nodes[goal]['cost'], path]
        return paths


def construction_tree(subtree, x_new, label_new, buchi_graph, centers, h_task, flag, connect, obs_check):

    # extend and rewire
    # successor root of current root
    curr = None
    for v in h_task.nodes:
        if v.x == subtree.init[0] and v.q == subtree.init[1]:
            curr = v
            break
    succ = [sc.xq() for sc in h_task.succ[curr]]
    # iterate over each buchi state
    for b_state in buchi_graph.nodes():

        # new product state
        q_new = (x_new, b_state)
        added = 0
        if q_new not in subtree.tree.nodes():
            # candidate parent state
            prec = subtree.prec(q_new, label_new, obs_check)
            # extend
            # no need to consider update the cost and acc of other subtrees since connect_root handles it
            # succ = set(todo_succ.keys())
            # succ.remove(subtree.init)
            added = subtree.extend(q_new, prec, succ, label_new)

        # rewire
        if q_new in subtree.tree.nodes():
            # print(q_new)
            # only rewire within the subtree, it may affect the node which is a root
            succ = subtree.succ(q_new, label_new, obs_check)
            subtree.rewire(q_new, succ)

            if flag:
                construction_tree_connect_root(subtree, q_new, label_new, centers, h_task, connect, obs_check)


def construction_tree_connect_root(subtree, q_new, label, centers, h_task, connect, obs_check):
    # extend towards to other roots
    curr = None
    for v in h_task.nodes:
        if v.x == subtree.tree.graph['init'][0] and v.q == subtree.tree.graph['init'][1]:
            curr = v
            break

    for sc in h_task.succ[curr]:
        if sc == curr:
            continue
        succ = sc.xq()
        # label of succ
        label_succ = None
        for l, coord in centers.items():
            if coord == succ[0]:
                label_succ = l + '_' + str(1)
                break
        # connect q_new to succ
        if subtree.obs_check(q_new, succ[0], label_succ, obs_check) and subtree.checkTranB(q_new[1], label, succ[1]):
            c = subtree.tree.nodes[q_new]['cost'] + np.abs(q_new[0][0]-succ[0][0]) + np.abs(q_new[0][1]-succ[0][1])
            # in the tree
            if succ in subtree.tree.nodes:
                delta_c = subtree.tree.nodes[succ]['cost'] - c
                # update the cost of node in the subtree rooted at near_vertex
                if delta_c > 0:
                    subtree.tree.remove_edge(list(subtree.tree.pred[succ].keys())[0], succ)
                    subtree.tree.nodes[succ]['acc'] = set(subtree.acpt_check(q_new, succ)[0])
                    subtree.tree.add_edge(q_new, succ)

            # not in the tree
            else:
                subtree.tree.add_node(succ, cost=c, label=label_succ, acc=set(subtree.acpt_check(q_new, succ)[0]))
                subtree.tree.add_edge(q_new, succ)
                # keep track of connection
                connect.add((curr, sc))


def multi_trees(h_task, buchi_graph, ts, centers, max_node, num):
    multi_tree = list()
    # a list of subtrees
    # for root in h_task.nodes():
    #     init = root.xq()
    #     multi_tree.append(tree(ts, buchi_graph, init))
    # better way to construct the roots of multitrees
    roots = set()
    for root in h_task.edges():
        init = root[0].xq()
        if init not in roots:
            roots.add(init)
            multi_tree.append(tree(ts, buchi_graph, init, 0))
    # =====================================================================================
    # n_max = n_max
    # c = 0
    # connect = set()
    # # for n in range(n_max):
    # now = datetime.now()
    # while np.sum([t.tree.number_of_nodes() for t in multi_tree]) < max_node:
    #     # print(n)
    #     i = random.randint(0, len(multi_tree)-1)
    #     # sample
    #     x_rand = multi_tree[i].sample()
    #     # if n <= c * n_max:
    #     # construction_tree(multi_tree[i], x_rand, buchi_graph, centers, h_task, 0, connect)
    #     # else:
    #     construction_tree(multi_tree[i], x_rand, buchi_graph, centers, h_task, 1, connect)
    #
    # end2path = dict()
    # for pair in connect:
    #     for t in range(len(multi_tree)):
    #         if multi_tree[t].tree.graph['init'] == (pair[0].x, pair[0].q):
    #             end2path[pair] = multi_tree[t].findpath([(pair[1].x, pair[1].q)])[0][1]
    #             break
    # num_path_seq = len(end2path.keys())
    # time1 = (datetime.now() - now).total_seconds()
    #
    # # option 2 parallel
    # multi_tree = list()
    # # a list of subtrees
    # for root in h_task.nodes:
    #     if 'accept' not in root.q:
    #         init = (root.x, root.q)
    #         multi_tree.append(tree(ts, buchi_graph, init, 0.25, no))
    #
    # # ====================================================================================
    c = 0
    k = 0
    connect = set()
    # for n in range(n_max):
    now = datetime.now()
    obs_check_dict = {}
    while np.sum([t.tree.number_of_nodes() for t in multi_tree]) < max_node:

        if int(np.sum([t.tree.number_of_nodes() for t in multi_tree]) / 1000) > k:
            print(np.sum([t.tree.number_of_nodes() for t in multi_tree]))
            k += 1
        x_new = multi_tree[np.random.randint(len(multi_tree), size=1)[0]].sample(num)
        label = multi_tree[0].label(x_new)
        if 'o' in label:
            continue
        if label != '':
            label = label + '_' + str(1)

        for i in range(len(multi_tree)):
            # if n <= c * n_max:
            if np.sum([t.tree.number_of_nodes() for t in multi_tree]) < c * max_node:  # 0 is better
                construction_tree(multi_tree[i], x_new, label, buchi_graph, centers, h_task, 0, connect, obs_check_dict)
            else:
                construction_tree(multi_tree[i], x_new, label, buchi_graph, centers, h_task, 1, connect, obs_check_dict)
    print(len(connect))
    k = 0
    end2path = dict()
    for pair in connect:
        k = k + 1
        print(k)
        for t in range(len(multi_tree)):
            if multi_tree[t].tree.graph['init'] == pair[0].xq():
                end2path[pair] = multi_tree[t].findpath([pair[1].xq()])[0][1]
                break

    time2 = (datetime.now() - now).total_seconds()
    print(time2, h_task.number_of_edges(), len(end2path.keys()))
    # print(time1, time2, num_path_seq, num_path_par)
    return end2path
