"""
__author__ = chrislaw
__project__ = TransferLTL
__date__ = 2/1/19
"""

from sympy import satisfiable
from networkx.classes.digraph import DiGraph
from sympy.logic.boolalg import to_dnf
from sympy.logic.boolalg import Equivalent, Not, And
from state import State


def target(exp, regions):
    """
    find the target location
    :param label: word
    :return: target location/region
    """

    target = []

    if exp != to_dnf('1'):
        # a set of target points  l3|l4
        for dnf in exp.args or [exp]:

            truth_table = satisfiable(dnf, algorithm="dpll")
            # find the target point with true value
            for key, value in truth_table.items():
                if value:
                    des = key.name.split('_')[0]
                    target.append(regions[des])
        return target


def match(h_task, curr, cand):
    """
    match if the label of the second state equals
    :param h_task:
    :param curr: parent node
    :param cand: child node
    :return: yes if there exists the same subtask
    """
    for edge in h_task.edges():
        if (edge[0].x == curr.x and edge[1].x == cand.x) or (edge[1].x == curr.x and edge[0].x == cand.x) \
                and Equivalent(edge[1].label, cand.label):
            return True
    return False


def hoftask(init, buchi_graph, regions, r):

    h_task = DiGraph(type='subtask')
    try:
        label = to_dnf(buchi_graph.edges[(buchi_graph.graph['init'][0], buchi_graph.graph['init'][0])]['label'], True)
    except KeyError:
        # no self loop
        label = to_dnf('0')
    init_node = State((init, buchi_graph.graph['init'][0]), label)
    h_task.add_node(init_node)
    open_set = list()
    open_set.append((init_node, '0'))
    explore_set = list()
    # if  q1 --label--> q2, [target(label), q1]:q2
    succ_dict = dict()
    succ_dict[(init_node, '0')] = {buchi_graph.graph['init'][0]}

    while open_set:
        curr = open_set.pop(0)
        # buchi state that curr corresponds to
        for q_target in succ_dict[curr]:
            try:
                label = to_dnf(buchi_graph.edges[(q_target, q_target)]['label'], True)
            except KeyError:
                # no self loop
                label = to_dnf('0')
            for q_b in buchi_graph.succ[q_target]:

                if q_b != q_target:

                    # region corresponding to the label/word
                    edge_label = (buchi_graph.edges[(q_target, q_b)]['label'])
                    if edge_label == '(1)':
                        x_set = [curr[0].x]
                    elif '~' in edge_label and to_dnf(edge_label).nargs == {1}:  # ~l3_1
                        if curr[0].x == regions[to_dnf(edge_label).args[0].name.split('_')[0]]:
                            x_set = [(curr[0].x[0], curr[0].x[1]-r)]
                        else:
                            x_set = [curr[0].x]
                    else:
                        x_set = target(to_dnf(edge_label), regions)
                    if x_set:
                        for x in x_set:
                            cand = State((x, q_target), label)
                            # seems no need to check
                            # if curr[0] != cand and not match(h_task, curr[0], cand):
                            if not match(h_task, curr[0], cand):
                                h_task.add_edge(curr[0], cand)

                            try:
                                succ_dict[(cand, edge_label)].add(q_b)
                            except KeyError:
                                succ_dict[(cand, edge_label)] = {q_b}

                            if (cand, edge_label) not in open_set and (cand, edge_label) not in explore_set:
                                open_set.append((cand, edge_label))
                                h_task.add_node(cand)

            explore_set.append(curr)

    return h_task