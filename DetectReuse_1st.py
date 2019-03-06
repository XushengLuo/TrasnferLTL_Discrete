from sympy import satisfiable
from networkx.classes.digraph import DiGraph
from sympy.logic.boolalg import to_dnf
from sympy.logic.boolalg import Not, And, Equivalent
from state import State
from DetermineRoots_1st import target
from sympy import Symbol
# def find_node(cand, h_task):
#     """
#
#     :param cand:
#     :param h_task:
#     :return:
#     """
#     for node in h_task.nodes:
#         if cand == node:
#             return node
#     return cand


def hoftask_no_simplified(init, buchi_graph, regions, r):
    try:
        label = to_dnf(buchi_graph.edges[(buchi_graph.graph['init'][0], buchi_graph.graph['init'][0])]['label'], True)
    except KeyError:
        # no self loop
        label = to_dnf('0')
    init_node = State((init, buchi_graph.graph['init'][0]), label)
    h_task = DiGraph(type='subtask', init=init_node)
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
                        # trick
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
                            # if curr[0] != cand:
                            h_task.add_edge(curr[0], cand)

                            # successor of current state and corresponding edge label
                            try:
                                succ_dict[(cand, edge_label)].add(q_b)
                            except KeyError:
                                succ_dict[(cand, edge_label)] = {q_b}

                            if (cand, edge_label) not in open_set and (cand, edge_label) not in explore_set:
                                open_set.append((cand, edge_label))
                                h_task.add_node(cand)

            explore_set.append(curr)

    return h_task


def inclusion(subtask_lib, subtask_new, tree, end2path):
    """
    whether subtask_lib is included in subtask_new
    :param subtask_lib:
    :param subtask_new:
    :return:
    """
    # trick
    # no need to build subtree for those who doesn't have self-loop and edge label != 1   []<> ( <>)
    if subtask_new[1].label == to_dnf('0') and subtask_new[0].x != subtask_new[1].x:
        return 'zero'
    condition = not satisfiable(And(subtask_lib[1].label, Not(subtask_new[1].label)))
    if subtask_lib[0].x == subtask_new[0].x and subtask_lib[1].x == subtask_new[1].x:
        condition = condition or good_execution(end2path[(subtask_lib[0], subtask_lib[1])], subtask_new[1].label, tree)
        if condition:
            return 'forward'
    elif subtask_lib[0].x == subtask_new[1].x and subtask_lib[1].x == subtask_new[0].x:
        condition = condition or good_execution(end2path[(subtask_lib[0], subtask_lib[1])],
                                                subtask_new[1].label, tree)
        if condition:
            return 'backward'

    return ''


def good_execution(path, selfloop, tree):
    """

    :param path:
    :param selfloop:
    :param tree:
    :return:
    """
    if not selfloop:
        return False
    # trick
    for point in path[:-1]:  # since the last point leads to transition
        label = tree.label(point[0])
        if not label:
            continue
        else:
            label = label + '_' + str(1)

        model = {a: False for a in selfloop.atoms()}
        model[Symbol(label)] = True
        if not selfloop.subs(model):
            return False

    return True


def match(subtask_lib, subtask_new):
    """
    whether subtask_lib equals subtask_new
    :param subtask_lib:
    :param subtask_new:
    :return:
    """
    if Equivalent(subtask_lib[1].label, subtask_new[1].label):
        if subtask_lib[0].x == subtask_new[0].x and subtask_lib[1].x == subtask_new[1].x:
            return 'forward'
        elif subtask_lib[0].x == subtask_new[1].x and subtask_lib[1].x == subtask_new[0].x:
            return 'backward'

    return ''


def replace(init_q, end_q, init_lib_q, end_lib_q, path, direction):
    """
    replace buchi states in the original lib subtask with new buchi states
    :param init_q:
    :param end_q:
    :param init_lib_q:
    :param end_lib_q:
    :param path:
    :param direction:
    :return:
    """

    assert init_lib_q == path[0][1], 'match error'
    assert end_lib_q == path[-1][1], 'match error'

    repath = []
    if direction == 'forward':
        repath.append((path[0][0], init_q))
        for point in path[1:]:
            repath.append((point[0], end_q))

    elif direction == 'backward':
        # reverse
        # initial state
        p = path[::-1]
        repath.append((p[0][0], init_q))

        for point in p[1:]:
                repath.append((point[0], end_q))
    return repath


def to_do(todo, newsubtask2subtask, subtask_new):
    """
    build a set of roots for the remaining tasks
    :param todo:
    :param newsubtask2subtask
    :param subtask_new:
    :return:
    """
    added = False
    for subtask in todo:
        mtch = match(subtask, subtask_new)
        if mtch:
            if (subtask[0].xq(), subtask[1].xq()) not in newsubtask2subtask.keys():
                newsubtask2subtask[(subtask[0].xq(), subtask[1].xq())] \
                    = [(subtask_new[0].xq(), subtask_new[1].xq(), mtch)]
            else:
                newsubtask2subtask[(subtask[0].xq(), subtask[1].xq())].append(
                    (subtask_new[0].xq(), subtask_new[1].xq(), mtch))
            added = True
    if not added:
        todo.add(subtask_new)


def detect_reuse(h_task_lib, h_task_new, end2path, tree):
    """
    detect resuable subtask
    :param h_task_lib:
    :param h_task_new:
    :param end2path:
    :return:
    """
    subtask2path = dict()
    todo = set()
    newsubtask2subtask = dict()

    # each edge in new h_task
    for subtask_new in h_task_new.edges:
        # match edges in library of h_task
        reused = False
        # (x, q0)  (x, q1) for now it's better to use this
        # trick
        if subtask_new[0].x == subtask_new[1].x:
            subtask2path[(subtask_new[0].xq(), subtask_new[1].xq())] = [subtask_new[0].xq(), subtask_new[1].xq()]
            continue

        for subtask_lib in h_task_lib.edges:
            # future work: it's better to compare the lib of controller
            if subtask_lib not in end2path.keys():
                continue
            direction = inclusion(subtask_lib, subtask_new, tree, end2path)
            if direction == 'forward':
                subtask2path[(subtask_new[0].xq(), subtask_new[1].xq())] \
                    = replace(subtask_new[0].q, subtask_new[1].q, subtask_lib[0].q, subtask_lib[1].q,
                              end2path[subtask_lib], 'forward')
                reused = True
                break
            elif direction == 'backward':
                subtask2path[(subtask_new[0].xq(), subtask_new[1].xq())] \
                    = replace(subtask_new[0].q, subtask_new[1].q, subtask_lib[0].q, subtask_lib[1].q,
                              end2path[subtask_lib], 'backward')
                reused = True
                break
            # trick: better
            elif direction == 'zero':  # no self loop hard to transit
                reused = True
                break
        # find no matching subtasks
        if not reused:
            to_do(todo, newsubtask2subtask, subtask_new)

    # starting point : subtask starting from starting point
    starting2waypoint = {key[0]: set() for key, _ in subtask2path.items()}
    for key, _ in subtask2path.items():
        starting2waypoint[key[0]].add(key)
    # including accepting state
    acpt = set()
    for node in h_task_new.nodes:
        if 'accept' in node.q:
            acpt.add(node.xq())

    # successor root
    todo_succ = {td[0].xq(): set() for td in todo}

    for td in todo:
        td_succ = list(h_task_new.succ[td[0]])
        for t in todo:
            if t[0] is not td[0] and t[0] in td_succ:
                todo_succ[td[0].xq()].add(t[0].xq())

    init_node = h_task_new.graph['init']
    if init_node.xq() not in todo_succ:
        todo_succ[init_node.xq()] = set()
        td_succ = list(h_task_new.succ[init_node])
        for t in todo:
            if t[0] is not init_node.xq() and t[0] in td_succ:
                todo_succ[init_node.xq()].add(t[0].xq())

        # endpoint
        # todo_succ[td[0].xq()].add(td[1].xq())
    return subtask2path, starting2waypoint, todo, todo_succ, newsubtask2subtask, acpt
