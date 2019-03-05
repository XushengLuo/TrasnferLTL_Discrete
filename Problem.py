"""
__author__ = chrislaw
__project__ = TransferLTL
__date__ = 2/1/19
"""

from shapely.geometry import Point, Polygon
import sys


class problemFormulation(object):
    def __init__(self, case):
        # +----------------------------------------------+
        # |                                              |
        # |                 Problem 1                    |
        # |                                              |
        # +----------------------------------------------+


        # +-----+-----+-----+
        # |  l1 |     | l2  |
        # |     +-----+     |
        # |       l4        |
        # |             l3  |
        # |    +-------+    |
        # | l5 |       |    |
        # +----+-------+----+
        # l1: (0.2, 0.8)
        # l2: (0.8, 0.8)
        # l3: (0.8, 0.4)
        # l4: (0.4, 0.4)
        # l5: (0.1, 0.2)

        self.workspace = (1, 1)
        # !! no whitespace in atomic proposation      b:ball s:square
        r = 0.05  # float(sys.argv[2])
        mul = 0.05 / r
        self.num = 1 / r
        self.ap = {'l1', 'l2', 'l3', 'l4', 'l5', 'l6'}
        center = [(0.2, 0.8), (0.8, 0.8), (0.8, 0.4), (0.4, 0.4), (0.1, 0.2), (0.1, 0.5)]

        self.centers = {'l1': (center[0][0] + mul * r / 2, center[0][1] + mul * r / 2),
                        'l2': (center[1][0] + mul * r / 2, center[1][1] + mul * r / 2),
                        'l3': (center[2][0] + mul * r / 2, center[2][1] + mul * r / 2),
                        'l4': (center[3][0] + mul * r / 2, center[3][1] + mul * r / 2),
                        'l5': (center[4][0] + mul * r / 2, center[4][1] + mul * r / 2),
                        'l6': (center[5][0] + mul * r / 2, center[5][1] + mul * r / 2)
                        }

        self.regions = {'l1': Polygon([center[0], (center[0][0] + mul * r, center[0][1]),
                                       (center[0][0] + mul * r, center[0][1] + mul * r),
                                       (center[0][0], center[0][1] + mul * r)]),
                        'l2': Polygon([center[1], (center[1][0] + mul * r, center[1][1]),
                                       (center[1][0] + mul * r, center[1][1] + mul * r),
                                       (center[1][0], center[1][1] + mul * r)]),
                        'l3': Polygon([center[2], (center[2][0] + mul * r, center[2][1]),
                                       (center[2][0] + mul * r, center[2][1] + mul * r),
                                       (center[2][0], center[2][1] + mul * r)]),
                        'l4': Polygon([center[3], (center[3][0] + mul * r, center[3][1]),
                                       (center[3][0] + mul * r, center[3][1] + mul * r),
                                       (center[3][0], center[3][1] + mul * r)]),
                        'l5': Polygon([center[4], (center[4][0] + mul * r, center[4][1]),
                                       (center[4][0] + mul * r, center[4][1] + mul * r),
                                       (center[4][0], center[4][1] + mul * r)]),
                        'l6': Polygon([center[5], (center[5][0] + mul * r, center[5][1]),
                                       (center[5][0] + mul * r, center[5][1] + mul * r),
                                       (center[5][0], center[5][1] + mul * r)])
                        }

        center = [(0.6, 0.3), (0.6, 0.5), (0.3, 0.6), (0.8, 0.6), (0.2, 0.3)]
        self.obs = {'o1': Polygon([(0.3, 0.0), (0.7, 0.0), (0.7, 0.2), (0.3, 0.2)]),
                    'o2': Polygon([(0.4, 0.7), (0.6, 0.7), (0.6, 1.0), (0.4, 1.0)]),
                    'o3': Polygon([(center[0][0], center[0][1]), (center[0][0] + mul * r, center[0][1]),
                                   (center[0][0] + mul * r, center[0][1] + mul * r),
                                   (center[0][0], center[0][1] + mul * r)]),
                    'o4': Polygon([(center[1][0], center[1][1]), (center[1][0] + mul * r, center[1][1]),
                                   (center[1][0] + mul * r, center[1][1] + mul * r),
                                   (center[1][0], center[1][1] + mul * r)]),
                    'o5': Polygon([(center[2][0], center[2][1]), (center[2][0] + mul * r, center[2][1]),
                                   (center[2][0] + mul * r, center[2][1] + mul * r),
                                   (center[2][0], center[2][1] + mul * r)]),
                    'o6': Polygon([(center[3][0], center[3][1]), (center[3][0] + mul * r, center[3][1]),
                                   (center[3][0] + mul * r, center[3][1] + mul * r),
                                   (center[3][0], center[3][1] + mul * r)]),
                    'o7': Polygon([(center[4][0], center[4][1]), (center[4][0] + mul * r, center[4][1]),
                                   (center[4][0] + mul * r, center[4][1] + mul * r),
                                   (center[4][0], center[4][1] + mul * r)])
                    }

        init_state = []
        case2robot = {0: 1,
                      1: 1,
                      2: 2,
                      21: 4,
                      3: 16,
                      4: 16,
                      5: 16,
                      6: 20,
                      7: 20}
        case = case  # int(sys.argv[1])
        for i in range(case2robot[case]):
            init_state.append((r / 2, r / 2))
        self.init_state = (r / 2, r / 2)
        # self.init_state = ((0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1))
        # self.init_state = ((0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1))
        # self.init_state = ((0.8, 0.1),(0.8, 0.1),(0.8, 0.1),(0.8, 0.1))
        # self.init_state = ((0.8, 0.1), (0.9, 0.1))
        # self.init_state = ((0.8, 0.1), )
        self.uni_cost = 0.1

        # #----------------------------------------------#
        # |                                              |
        # |                 Problem 2                    |
        # |                                              |
        # #----------------------------------------------#

        # +-----+-----+-----+
        # | r4,b|r5,rb| r6  |
        # +-----+-----+-----+
        # | c1  | c2  | c3  |
        # +-----+-----+-----+
        # | r1  | r2,b|r3,gb|
        # +-----+-----+-----+



        """
        +----------------------------+
        |   Propositonal Symbols:    |
        |        true, false         |
        |	    any lowercase string |
        |                            |
        |   Boolean operators:       |
        |       !   (negation)       |
        |       ->  (implication)    |
        |	    <-> (equivalence)    |
        |       &&  (and)            |
        |       ||  (or)             |
        |                            |
        |   Temporal operators:      |
        |       []  (always)         |
        |       <>  (eventually)     |
        |       U   (until)          |
        |       V   (release)        |
        |       X   (next)           |
        +----------------------------+
        """

        # self.formula = '<>(l3 && []<>l4)'
        # self.formula = '<> (l11) &&   [](<> ( l21 && <> (l31 && <> l41 ) ) )'

        if case == 0:
            # self.formula = '<> e1 && <> e2'
            # self.formula = '<> e1 && !e1 U e2 '
            #
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                     }
            # self.exclusion = [('e1', 'e2')]
            # self.no = []

            # self.formula = '<> e1 && <> e2'
            self.formula = '<> e1 && <> e2  && <> e3 && <> e4 && <> e5 && <> e6'

            self.formula_comp = {1: '(l1_1)',
                                 2: '(l2_1)',
                                 3: '(l3_1)',
                                 4: '(l4_1)',
                                 5: '(l5_1)',
                                 6: '(l6_1)'}
            self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
                              ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5'), ('e1', 'e6'), ('e2', 'e6'),
                              ('e3', 'e6'), ('e4', 'e6'), ('e5', 'e6')]

        # ---------------------------- case 1 --------------------------------
        if case == 1:
            # self.formula = ' [] (<> e1 && <> e2)'
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3')]
            # self.formula = '[] (<> e1 && <> e2 && <> e4 && <> e6)'
            #
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)',
            #                      4: '(l4_1)',
            #                      6: '(l3_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
            #                   ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5'), ('e1', 'e6'), ('e2', 'e6'),
            #                   ('e3', 'e6'), ('e4', 'e6'), ('e5', 'e6')]

            # self.formula = '<> e1 && <> e2 && []!e5'
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3')]
            # self.no = []
            #
            # self.formula = '<> e1 && <> e2 && <> e3 && <> e5'
            # self.formula_comp = {1: '(l1_1)',
            #                      2: '(l2_1)',
            #                      3: '(l5_1)',
            #                      4: '(l4_1)',
            #                      5: '(l6_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
            #                   ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5')]
            # self.no = []

            # #----------------------------------------------#
            # |              1: multisubtree                 |
            # |              2: uniform                      |
            # |                                              |
            # #----------------------------------------------#
            # self.formula = '[] (<> e1 && <> e4) && [] (e1 -> X (!e1 U e4))'
            # self.formula = '[]<> (e1 && <> e3) && !e1 U e3'
            # self.formula = '[]<> (e1 && <> e2)  && []<> (e3 && <> e4)  && !e1 U e3'
            # self.formula = '[] (<> e1 && <> e2) && [](<> e3 && <> e4) && <> e5 && <> e6  && [] (e1 -> X (!e1 U e2))'\
            #              '&& [] (e3 -> X (!e3 U e4))'
            # self.formula = '[] <> (e1 && <> e2) && []<>( e3 && <> e4) && <> e5  && <> e6 && !e1 U e5'
            # self.formula = '[]<> e4 && [](<> e3  && <> e1) && (!e1 U e2)  && []!e5'   # 1 2   without any reusable skills, 1 is still much better than 2
            # self.formula = '[]<> e1 && <> e2 && <> e3 && <> e4 && !e3 U e2'
            # test directreuse
            # self.formula = '[]<> e1  && !e1 U e3'
            # self.formula = '[](e1 -> X (!e1 U e5)) && []<>e1 && []<> e5'  # better than [](e1 -> X (!e1 U e5)) && []<>e1
            # self.formula = '[]<> e1 && [] <> e2'  # worse than '[]<> e1 && <> e5'
            # self.formula = '[]<> (e1 && <> (e3 && <> e2))'
            # self.formula = '[]<> (e1 && <> e3)'
            # self.formula = '[] (<> e1 && <> e2 && <> e3 ) && <> e5 && !e1 U e2 && !e3 U e2'
            # self.formula = '[] (<> e1 && <> e2  && <> e3  && <> e4 && <> e5 && <> e6)'
            self.formula_comp = {1: '(l1_1)',
                                 2: '(l2_1)',
                                 3: '(l3_1)',
                                 4: '(l4_1)',
                                 5: '(l5_1)',
                                 6: '(l6_1)'}
            self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e2', 'e3'), ('e1', 'e4'), ('e2', 'e4'), ('e3', 'e4'),
                              ('e1', 'e5'), ('e2', 'e5'), ('e3', 'e5'), ('e4', 'e5'), ('e1', 'e6'), ('e2', 'e6'),
                              ('e3', 'e6'), ('e4', 'e6'), ('e5', 'e6')]

            # self.formula = '<> e1 && <> (e2  && <> (e3 && <> ( e4 && <> e5)))'
            #
            # self.formula_comp = {1: '(l4_1)',
            #                          2: '(l3_1)',
            #                          3: '(l1_1)',
            #                          4: '(l2_1)',
            #                          5: '(l5_1)'}
            # self.exclusion = [('e1', 'e2'), ('e1', 'e3'), ('e1', 'e3'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'),
            #                       ('e2', 'e4'), ('e2', 'e5'), ('e3', 'e4'), ('e3', 'e5'), ('e4', 'e5')]
            # self.no = []

    def Formulation(self):
        # print('Task specified by LTL formula: ' + self.formula)
        return self.workspace, self.regions, self.centers, self.obs, self.init_state, self.uni_cost, self.formula, \
               self.formula_comp, self.exclusion, self.num
