from collections import OrderedDict
import numpy as np
from datetime import datetime
from networkx.classes.digraph import DiGraph
from shapely.geometry import Point, LineString
from networkx.algorithms import dfs_labeled_edges
from numpy.random import uniform
from ShortestPath import build_graph, shortest_path
from Problem import problemFormulation
import Buchi
from Visualization import path_plot


class tree(object):
    """ construction of prefix and suffix tree
    """

    def __init__(self, ts, buchi_graph, init, final, seg, env):
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
        self.tree.add_node(init, cost=0, label=label)

        self.group = dict()
        self.add_group(init)

        self.b_final = final
        self.seg = seg

        self.p = 0.9

        self.env = env

    def add_group(self, q_state):
        """
        group nodes with same buchi state
        :param q_state: new state added to the tree
        """
        try:
            self.group[q_state[1]].append(q_state)
        except KeyError:
            self.group[q_state[1]] = [q_state]

    def min2final(self, min_qb_dict, b_final, cand):
        """
         collects the buchi state in the tree with minimum distance to the final state
        :param min_qb_dict: dict
        :param b_final: feasible final state
        :return: list of buchi states in the tree with minimum distance to the final state
        """
        l_min = np.inf
        b_min = []
        for b_state in cand:
            if min_qb_dict[(b_state, b_final)] < l_min:
                l_min = min_qb_dict[(b_state, b_final)]
                b_min = [b_state]
            elif min_qb_dict[(b_state, b_final)] == l_min:
                b_min.append(b_state)
        return b_min

    def all2one(self, b_min):
        """
        partition nodes into 2 groups
        :param b_min: buchi states with minimum distance to the finals state
        :return: 2 groups
        """
        q_min2final = []
        q_minNot2final = []
        for b_state in self.group.keys():
            if b_state in b_min:
                q_min2final = q_min2final + self.group[b_state]
            else:
                q_minNot2final = q_minNot2final + self.group[b_state]
        return q_min2final, q_minNot2final

    def sample(self, buchi_graph, min_qb_dict, num_grid, centers):
        """
        sample point from the workspace
        :return: sampled point, tuple
        """

        # collects the buchi state in the tree with minimum distance to the final state
        b_min = self.min2final(min_qb_dict, self.b_final, self.group.keys())
        # partition of nodes
        q_min2final, q_minNot2final = self.all2one(b_min)
        # sample random nodes
        p_rand = np.random.uniform(0, 1, 1)
        q_rand = []
        if (p_rand <= self.p and len(q_min2final) > 0) or not q_minNot2final:
            q_rand = q_min2final[np.random.randint(0, len(q_min2final))]
        elif p_rand > self.p or not q_min2final:
            q_rand = q_minNot2final[np.random.randint(0, len(q_minNot2final))]
        # find feasible succssor of buchi state in q_rand
        Rb_q_rand = []
        label = self.label(q_rand[0])
        if label != '':
            label = label + '_' + str(1)

        for b_state in buchi_graph.succ[q_rand[1]]:
            # if self.t_satisfy_b(x_label, buchi_graph.edges[(q_rand[1], b_state)]['label']):
            if self.t_satisfy_b_truth(label, buchi_graph.edges[(q_rand[1], b_state)]['truth']):
                Rb_q_rand.append(b_state)
        # if empty
        if not Rb_q_rand:
            return Rb_q_rand, Rb_q_rand
        # collects the buchi state in the reachable set of qb_rand with minimum distance to the final state
        b_min = self.min2final(min_qb_dict, self.b_final, Rb_q_rand)

        # collects the buchi state in the reachable set of b_min with distance to the final state equal to that of b_min - 1
        decr_dict = dict()
        for b_state in b_min:
            decr = []
            for succ in buchi_graph.succ[b_state]:
                if min_qb_dict[(b_state, self.b_final)] - 1 == min_qb_dict[(succ, self.b_final)] or succ == self.b_final:
                    decr.append(succ)
            decr_dict[b_state] = decr
        M_cand = [b_state for b_state in decr_dict.keys() if decr_dict[b_state]]
        # if empty
        if not M_cand:
            return M_cand, M_cand
        # sample b_min and b_decr
        b_min = M_cand[np.random.randint(0, len(M_cand))]
        b_decr = decr_dict[b_min][np.random.randint(0, len(decr_dict[b_min]))]

        truth = buchi_graph.edges[(b_min, b_decr)]['truth']
        x_rand = list(q_rand[0])
        return self.buchi_guided_sample_by_truthvalue(truth, x_rand, num_grid, centers)

    def buchi_guided_sample_by_truthvalue(self, truth, x_rand, num_grid, centers):
        """
        sample guided by truth value
        :param truth: the value making transition occur
        :param x_rand: random selected node
        :param x_label: label of x_rand
        :param regions: regions
        :return: new sampled point
        """
        # stay
        if truth == '1':
            return x_rand

        p_new = 0.8
        region = ''
        for key in truth:
            if truth[key]:
                region = key
        if not region:
            return x_rand

        # select second point withi high probability
        if uniform(0, 1, 1) < p_new:
            return self.target(centers, x_rand, region)
        else:
            while True:
                x_candidate = [0, 0]
                # parallel to one axis
                line = np.random.randint(2, size=1)[0]
                x_candidate[line] = x_rand[line]
                # sample another component
                r = round(1 / num_grid, 10)
                x_candidate[1 - line] = round(np.random.randint(num_grid, size=1)[0] * r + r / 2, 10)
                if x_candidate != x_rand:
                    if 'o' in self.label(x_candidate):
                        continue
                    return tuple(x_candidate)

    def target(self, centers, source, tg):
        source = source
        tg = centers[tg.split('_')[0]]
        destination = shortest_path(self.env, tuple(source), tg)
        return destination

    def acpt_check(self, q_new, label_new, obs_check):
        """
        check the accepting state in the patg leading to q_new
        :param q_min:
        :param q_new:
        :return:
        """
        if self.seg == 'pre':
            if 'accept' in q_new[1]:
                self.goals.append(q_new)
        elif self.seg == 'suf':
            if self.obs_check(self.init, q_new[0], label_new, obs_check) and self.checkTranB(q_new[1], label_new,
                                                                                             self.init[1]):
                self.goals.append(q_new)

    def extend(self, q_new, prec_list, label_new, obs_check):
        """
        :param: q_new: new state form: tuple (mulp, buchi)
        :param: near_v: near state form: tuple (mulp, buchi)
        :param: obs_check: check obstacle free  form: dict { (mulp, mulp): True }
        :param: succ: list of successor of the root
        :return: extending the tree
        """
        # if q_new == ((0.825, 0.425), 'T1_S1'):
        #     print('df')
        added = 0
        cost = np.inf
        q_min = ()
        for pre in prec_list:
            c = self.tree.nodes[pre]['cost'] + np.abs(q_new[0][0] - pre[0][0]) + np.abs(q_new[0][1] - pre[0][1])
            if c < cost:
                added = 1
                q_min = pre
                cost = c
        if added == 1:
            self.add_group(q_new)
            self.tree.add_node(q_new, cost=cost, label=label_new)
            self.tree.add_edge(q_min, q_new)
            self.acpt_check(q_new, label_new, obs_check)

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
            c = self.tree.nodes[q_new]['cost'] + np.abs(q_new[0][0] - suc[0][0]) + np.abs(q_new[0][1] - suc[0][1])
            delta_c = self.tree.nodes[suc]['cost'] - c
            # update the cost of node in the subtree rooted at near_vertex
            if delta_c > 0:
                self.tree.remove_edge(list(self.tree.pred[suc].keys())[0], suc)
                self.tree.add_edge(q_new, suc)
                edges = dfs_labeled_edges(self.tree, source=suc)
                for u, v, d in edges:
                    if d == 'forward':
                        self.tree.nodes[v]['cost'] = self.tree.nodes[v]['cost'] - delta_c

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
            if self.seg == 'suf':
                paths[i] = [self.tree.nodes[goal]['cost'] + np.abs(path[-1][0][0] - self.init[0][0]) + np.abs(
                    path[-1][0][1] - self.init[0][1]), path + [self.init]]
        return paths

    def path_cost(self, path):
        """
        calculate cost
        :param path:
        :return:
        """
        cost = 0
        for k in range(len(path) - 1):
            cost += np.abs(path[k + 1][0][0] - path[k][0][0]) + np.abs(path[k + 1][0][1] - path[k][0][1])
        return cost


def construction_tree(bias_tree, buchi_graph, obs_check, min_qb_dict, num_grid, centers):
    while bias_tree.tree.number_of_nodes() < 10000:

        x_new = bias_tree.sample(buchi_graph, min_qb_dict, num_grid, centers)
        if not x_new[0]:
            continue
        # sample
        label_new = bias_tree.label(x_new)
        if 'o' in label_new:
            continue
        if label_new != '':
            label_new = label_new + '_' + str(1)

        for b_state in buchi_graph.nodes():

            q_new = (x_new, b_state)

            if q_new not in bias_tree.tree.nodes():
                # candidate parent state
                prec = bias_tree.prec(q_new, label_new, obs_check)
                bias_tree.extend(q_new, prec, label_new, obs_check)
            # rewire
            if q_new in bias_tree.tree.nodes():
                # only rewire within the subtree, it may affect the node which is a root
                succ = bias_tree.succ(q_new, label_new, obs_check)
                bias_tree.rewire(q_new, succ)

            if len(bias_tree.goals) > 0:
                cost_pat = bias_tree.findpath(bias_tree.goals)
                return cost_pat


def bias_sampling_alg(buchi_graph, init, ts, centers, num_grid, obs_check, min_qb, env):
    """
    build multiple subtree by transferring
    :param todo: new subtask built from the scratch
    :param buchi_graph:
    :param init: root for the whole formula
    :param todo_succ: the successor of new subtask
    :param ts:
    :param no:
    :param centers:
    :param max_node: maximum number of nodes of all subtrees
    :param subtask2path: (init, end) --> init, p, ..., p, end
    :param starting2waypoint: init --> (init, end)
    :param newsubtask2subtask_p: new subtask need to be planned --> new subtask noneed
    :return:
    """
    start = datetime.now()
    bias_tree = tree(ts, buchi_graph, init,
                     buchi_graph.graph['accept'][np.random.randint(0, len(buchi_graph.graph['accept']))], 'pre', env)
    cost_path_pre = construction_tree(bias_tree, buchi_graph, obs_check, min_qb, num_grid, centers)
    pre_time = (datetime.now() - start).total_seconds()
    start = datetime.now()
    opt_cost = (np.inf, np.inf)
    opt_path_pre = []
    opt_path_suf = []

    for i in range(1):
        # goal product state
        goal = bias_tree.goals[i]
        tree_suf = tree(ts, buchi_graph, goal, goal[1], 'suf', env)

        label_new = bias_tree.label(goal)
        if 'o' in label_new:
            continue
        if label_new != '':
            label_new = label_new + '_' + str(1)
        if tree_suf.obs_check(tree_suf.init, tree_suf.init[0], label_new, obs_check) and tree_suf.checkTranB(
                tree_suf.init[1], label_new,
                tree_suf.init[1]):
            opt_path_pre = cost_path_pre[i][1]  # plan of [(position, buchi)]
            return pre_time, cost_path_pre[i][0], opt_path_pre

        # update accepting buchi state
        # buchi_graph.graph['accept'] = goal[1]
        # construct suffix tree
        cost_path_suf_cand = construction_tree(tree_suf, buchi_graph, obs_check, min_qb, num_grid, centers)

        # print('--------------suffix path for {0}-th goal (of {1} in total)---------------------'.format(i, len(tree_pre.goals)))
        # print('{0}-th goal: {1} accepting goals found'.format(i, len(tree_suf.goals)))
        # couldn't find the path
        try:
            # order according to cost
            cost_path_suf_cand = OrderedDict(sorted(cost_path_suf_cand.items(), key=lambda x: x[1][0]))
            mincost = list(cost_path_suf_cand.keys())[0]
        except IndexError:
            del cost_path_pre[i]
            # print('delete {0}-th item in cost_path_pre, {1} left'.format(i, len(cost_path_pre)))
            continue
        cost_path_suf = cost_path_suf_cand[mincost]

        if cost_path_pre[i][0] + cost_path_suf[0] < opt_cost[0] + opt_cost[1]:
            opt_path_pre = cost_path_pre[i][1]  # plan of [(position, buchi)]
            opt_path_suf = cost_path_suf[1]
            opt_cost = cost_path_pre[i][0] + cost_path_suf[0]  # optimal cost (pre_cost, suf_cost)

        suf_time = (datetime.now() - start).total_seconds()
        return pre_time + suf_time, opt_cost, opt_path_pre + opt_path_suf

        # path_plot(opt_path_pre+opt_path_suf, regions, obs, num_grid)
        # path_plot(opt_path_,suf, regions, obs, num_grid)


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

t = tree(ts, buchi_graph, (init_state, buchi_graph.graph['init'][0]), '', '', [])
env = build_graph(num_grid, t)

time = []
cost = []
opt_path = []
opt_cost = np.inf
for i in range(10):
    tm, c, p = bias_sampling_alg(buchi_graph, (init_state, buchi_graph.graph['init'][0]), ts, centers, num_grid, dict(),
                                 min_qb, env)
    print(tm, c)
    time.append(tm)
    cost.append(c)
    if c < opt_cost:
        opt_cost = c
        opt_path = p

print(np.mean(time), np.mean(cost), opt_cost)
path_plot(opt_path, regions, obs, num_grid)
