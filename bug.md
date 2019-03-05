sweep task: transfer_tree.used.add(ending)

constuct tree sample funciton :         if k == len(sample_list) - 2:
            construction_tree_connect_root(subtree, sample_list[k],
                                                   subtree.tree.nodes[sample_list[k]]['label'],
                                                   centers, todo_succ, connect,
                                                   starting2waypoint, subtask2path, newsubtask2subtask_p, root2index,
                                                   root_pred2index_node, index2node_root, multi_tree, init, obs_check)
