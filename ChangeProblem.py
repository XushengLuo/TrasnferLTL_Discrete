"""
__author__ = chrislaw
__project__ = TransferLTL
__date__ = 2/1/19
"""

from shapely.geometry import Point, Polygon
import sys
import random
import numpy
import math


def round_down(n, decimals=0):
    multiplier = 10 ** decimals
    mid = math.floor(n * multiplier) / multiplier
    if n - mid >= 0.05:
        mid = round(mid + 0.05, 2)
    return mid


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

        self.centers = {'l1': (round(center[0][0] + mul * r / 2, 10), round(center[0][1] + mul * r / 2, 10)),
                        'l2': (round(center[1][0] + mul * r / 2, 10), round(center[1][1] + mul * r / 2, 10)),
                        'l3': (round(center[2][0] + mul * r / 2, 10), round(center[2][1] + mul * r / 2, 10)),
                        'l4': (round(center[3][0] + mul * r / 2, 10), round(center[3][1] + mul * r / 2, 10)),
                        'l5': (round(center[4][0] + mul * r / 2, 10), round(center[4][1] + mul * r / 2, 10)),
                        'l6': (round(center[5][0] + mul * r / 2, 10), round(center[5][1] + mul * r / 2, 10))
                        }

        self.regions = {'l1': Polygon([center[0], (round(center[0][0] + mul * r, 10), round(center[0][1], 10)),
                                       (round(center[0][0] + mul * r, 10), round(center[0][1] + mul * r, 10)),
                                       (round(center[0][0], 10), round(center[0][1] + mul * r, 10))]),
                        'l2': Polygon([center[1], (round(center[1][0] + mul * r, 10), round(center[1][1], 10)),
                                       (round(center[1][0] + mul * r, 10), round(center[1][1] + mul * r, 10)),
                                       (round(center[1][0], 10), round(center[1][1] + mul * r, 10))]),
                        'l3': Polygon([center[2], (round(center[2][0] + mul * r, 10), round(center[2][1], 10)),
                                       (round(center[2][0] + mul * r, 10), round(center[2][1] + mul * r, 10)),
                                       (round(center[2][0], 10), round(center[2][1] + mul * r, 10))]),
                        'l4': Polygon([center[3], (round(center[3][0] + mul * r, 10), round(center[3][1], 10)),
                                       (round(center[3][0] + mul * r, 10), round(center[3][1] + mul * r, 10)),
                                       (round(center[3][0], 10), round(center[3][1] + mul * r, 10))]),
                        'l5': Polygon([center[4], (round(center[4][0] + mul * r, 10), round(center[4][1], 10)),
                                       (round(center[4][0] + mul * r, 10), round(center[4][1] + mul * r, 10)),
                                       (round(center[4][0], 10), round(center[4][1] + mul * r, 10))]),
                        'l6': Polygon([center[5], (round(center[5][0] + mul * r, 10), round(center[5][1], 10)),
                                       (round(center[5][0] + mul * r, 10), round(center[5][1] + mul * r, 10)),
                                       (round(center[5][0], 10), round(center[5][1] + mul * r, 10))])
                        }

        obs_center = [(0.6, 0.3), (0.6, 0.5), (0.3, 0.6), (0.8, 0.6), (0.2, 0.3), (0.4, 0.5), (0.1, 0.7),  # (0.1, 0.8),
                      (0.4, 0.2), (0.5, 0.4), (0.2, 0.4), (0.7, 0.7)]
        self.obs = {'o1': Polygon([(0.3, 0.0), (0.7, 0.0), (0.7, 0.2), (0.3, 0.2)]),
                    'o2': Polygon([(0.4, 0.7), (0.6, 0.7), (0.6, 1.0), (0.4, 1.0)]),  # less
                    'o3': Polygon([(round(obs_center[0][0], 10), round(obs_center[0][1], 10)),
                                   (round(obs_center[0][0] + mul * r, 10), round(obs_center[0][1], 10)),
                                   (round(obs_center[0][0] + mul * r, 10), round(obs_center[0][1] + mul * r, 10)),
                                   (round(obs_center[0][0], 10), round(obs_center[0][1] + mul * r, 10))]),
                    'o4': Polygon([(round(obs_center[1][0], 10), round(obs_center[1][1], 10)),
                                   (round(obs_center[1][0] + mul * r, 10), round(obs_center[1][1], 10)),
                                   (round(obs_center[1][0] + mul * r, 10), round(obs_center[1][1] + mul * r, 10)),
                                   (round(obs_center[1][0], 10), round(obs_center[1][1] + mul * r, 10))]),
                    'o5': Polygon([(round(obs_center[2][0], 10), round(obs_center[2][1], 10)),
                                   (round(obs_center[2][0] + mul * r, 10), round(obs_center[2][1], 10)),
                                   (round(obs_center[2][0] + mul * r, 10), round(obs_center[2][1] + mul * r, 10)),
                                   (round(obs_center[2][0], 10), round(obs_center[2][1] + mul * r, 10))]),
                    'o6': Polygon([(round(obs_center[3][0], 10), round(obs_center[3][1], 10)),
                                   (round(obs_center[3][0] + mul * r, 10), round(obs_center[3][1], 10)),
                                   (round(obs_center[3][0] + mul * r, 10), round(obs_center[3][1] + mul * r, 10)),
                                   (round(obs_center[3][0], 10), round(obs_center[3][1] + mul * r, 10))]),
                    'o7': Polygon([(round(obs_center[4][0], 10), round(obs_center[4][1], 10)),
                                   (round(obs_center[4][0] + mul * r, 10), round(obs_center[4][1], 10)),
                                   (round(obs_center[4][0] + mul * r, 10), round(obs_center[4][1] + mul * r, 10)),
                                   (round(obs_center[4][0], 10), round(obs_center[4][1] + mul * r, 10))]),  # more
                    'o8': Polygon([(round(obs_center[5][0], 10), round(obs_center[5][1], 10)),
                                   (round(obs_center[5][0] + mul * r, 10), round(obs_center[5][1], 10)),
                                   (round(obs_center[5][0] + mul * r, 10), round(obs_center[5][1] + mul * r, 10)),
                                   (round(obs_center[5][0], 10), round(obs_center[5][1] + mul * r, 10))]),
                    'o9': Polygon([(round(obs_center[6][0], 10), round(obs_center[6][1], 10)),
                                   (round(obs_center[6][0] + mul * r, 10), round(obs_center[6][1], 10)),
                                   (round(obs_center[6][0] + mul * r, 10), round(obs_center[6][1] + mul * r, 10)),
                                   (round(obs_center[6][0], 10), round(obs_center[6][1] + mul * r, 10))]),
                    'o10': Polygon([(round(obs_center[7][0], 10), round(obs_center[7][1], 10)),
                                    (round(obs_center[7][0] + mul * r, 10), round(obs_center[7][1], 10)),
                                    (round(obs_center[7][0] + mul * r, 10), round(obs_center[7][1] + mul * r, 10)),
                                    (round(obs_center[7][0], 10), round(obs_center[7][1] + mul * r, 10))]),  # more more
                    'o11': Polygon([(round(obs_center[8][0], 10), round(obs_center[8][1], 10)),
                                    (round(obs_center[8][0] + mul * r, 10), round(obs_center[8][1], 10)),
                                    (round(obs_center[8][0] + mul * r, 10), round(obs_center[8][1] + mul * r, 10)),
                                    (round(obs_center[8][0], 10), round(obs_center[8][1] + mul * r, 10))]),  # more more
                    'o12': Polygon([(round(obs_center[9][0], 10), round(obs_center[9][1], 10)),
                                    (round(obs_center[9][0] + mul * r, 10), round(obs_center[9][1], 10)),
                                    (round(obs_center[9][0] + mul * r, 10), round(obs_center[9][1] + mul * r, 10)),
                                    (round(obs_center[9][0], 10), round(obs_center[9][1] + mul * r, 10))]),  # more more
                    'o13': Polygon([(round(obs_center[10][0], 10), round(obs_center[10][1], 10)),
                                    (round(obs_center[10][0] + mul * r, 10), round(obs_center[10][1], 10)),
                                    (round(obs_center[10][0] + mul * r, 10), round(obs_center[10][1] + mul * r, 10)),
                                    (round(obs_center[10][0], 10), round(obs_center[10][1] + mul * r, 10))])
                    # more more

                    }
        #  '[] (<> e1 && <> e2)'
        # self.path = [((0.025, 0.025), 'T0_init'), ((0.075, 0.025), 'T0_init'), ((0.075, 0.825), 'T0_init'),
        #         ((0.225, 0.825), 'T0_init'), ((0.375, 0.825), 'T1_S1'), ((0.375, 0.675), 'T1_S1'),
        #         ((0.775, 0.675), 'T1_S1'), ((0.775, 0.775), 'T1_S1'), ((0.825, 0.775), 'T1_S1'),
        #         ((0.825, 0.825), 'T1_S1'),
        #         ((0.825, 0.825), 'accept_S1'), ((0.825, 0.825), 'accept_S1'), ((0.825, 0.775), 'T0_init'),
        #         ((0.775, 0.775), 'T0_init'), ((0.775, 0.675), 'T0_init'), ((0.425, 0.675), 'T0_init'),
        #         ((0.225, 0.675), 'T0_init'), ((0.225, 0.825), 'T0_init'), ((0.325, 0.825), 'T1_S1'),
        #         ((0.325, 0.675), 'T1_S1'), ((0.775, 0.675), 'T1_S1'), ((0.775, 0.775), 'T1_S1'),
        #         ((0.825, 0.775), 'T1_S1'),
        #         ((0.825, 0.825), 'T1_S1'), ((0.825, 0.825), 'accept_S1')]
        #  '[] (<> e1 && <> e4) && [] (e1 -> X (!e1 U e4))'
        # self.path = [((0.025, 0.025), 'T0_init'), ((0.025, 0.825), 'T0_init'), ((0.225, 0.825), 'T0_init'),
        #              ((0.225, 0.525), 'T1_S6'), ((0.275, 0.525), 'T1_S6'), ((0.275, 0.425), 'T1_S6'),
        #              ((0.425, 0.425), 'T1_S6'), ((0.425, 0.425), 'accept_S6'), ((0.425, 0.425), 'accept_S6'),
        #              ((0.375, 0.425), 'T0_init'), ((0.375, 0.675), 'T0_init'), ((0.225, 0.675), 'T0_init'),
        #              ((0.225, 0.825), 'T0_init'), ((0.225, 0.475), 'T1_S6'), ((0.275, 0.475), 'T1_S6'),
        #              ((0.275, 0.425), 'T1_S6'), ((0.425, 0.425), 'T1_S6'), ((0.425, 0.425), 'accept_S6')]
        # [] (<> e1 && <> e2  && <> e3  && <> e4 && <> e5 && <> e6)
        # self.path = [((0.025, 0.025), 'T0_init'), ((0.075, 0.025), 'T0_init'), ((0.075, 0.075), 'T0_init'),
        #              ((0.175, 0.075), 'T0_init'), ((0.175, 0.825), 'T0_init'), ((0.225, 0.825), 'T0_init'),
        #              ((0.325, 0.825), 'T1_S1'), ((0.325, 0.675), 'T1_S1'), ((0.775, 0.675), 'T1_S1'),
        #              ((0.775, 0.775), 'T1_S1'), ((0.825, 0.775), 'T1_S1'), ((0.825, 0.825), 'T1_S1'),
        #              ((0.825, 0.775), 'T2_S1'), ((0.775, 0.775), 'T2_S1'), ((0.775, 0.475), 'T2_S1'),
        #              ((0.825, 0.475), 'T2_S1'), ((0.825, 0.425), 'T2_S1'), ((0.825, 0.475), 'T3_S1'),
        #              ((0.775, 0.475), 'T3_S1'), ((0.475, 0.475), 'T3_S1'), ((0.475, 0.425), 'T3_S1'),
        #              ((0.425, 0.425), 'T3_S1'), ((0.375, 0.425), 'T4_S1'), ((0.325, 0.425), 'T4_S1'),
        #              ((0.275, 0.425), 'T4_S1'), ((0.275, 0.275), 'T4_S1'), ((0.225, 0.275), 'T4_S1'),
        #              ((0.225, 0.225), 'T4_S1'), ((0.125, 0.225), 'T4_S1'), ((0.125, 0.525), 'T5_S1'),
        #              ((0.125, 0.525), 'accept_S1'), ((0.125, 0.525), 'accept_S1'), ((0.225, 0.525), 'T0_init'),
        #              ((0.225, 0.825), 'T0_init'), ((0.325, 0.825), 'T1_S1'), ((0.325, 0.675), 'T1_S1'),
        #              ((0.775, 0.675), 'T1_S1'), ((0.775, 0.775), 'T1_S1'), ((0.825, 0.775), 'T1_S1'),
        #              ((0.825, 0.825), 'T1_S1'), ((0.825, 0.775), 'T2_S1'), ((0.775, 0.775), 'T2_S1'),
        #              ((0.775, 0.475), 'T2_S1'), ((0.825, 0.475), 'T2_S1'), ((0.825, 0.425), 'T2_S1'),
        #              ((0.825, 0.475), 'T3_S1'), ((0.475, 0.475), 'T3_S1'), ((0.475, 0.425), 'T3_S1'),
        #              ((0.425, 0.425), 'T3_S1'), ((0.375, 0.425), 'T4_S1'), ((0.325, 0.425), 'T4_S1'),
        #              ((0.275, 0.425), 'T4_S1'), ((0.275, 0.275), 'T4_S1'), ((0.225, 0.275), 'T4_S1'),
        #              ((0.225, 0.225), 'T4_S1'), ((0.125, 0.225), 'T4_S1'), ((0.125, 0.525), 'T5_S1'),
        #              ((0.125, 0.525), 'accept_S1')]
        # []<> (e1 && <> e2)  && []<> (e3 && <> e4)  && !e1 U e3'
        # self.path = [((0.025, 0.025), 'T0_init'), ((0.275, 0.025), 'T1_S1'), ((0.275, 0.275), 'T1_S1'),
        #              ((0.775, 0.275), 'T1_S1'), ((0.775, 0.375), 'T1_S1'), ((0.825, 0.375), 'T1_S1'),
        #              ((0.825, 0.425), 'T1_S1'), ((0.825, 0.475), 'T1_S13'), ((0.525, 0.475), 'T1_S13'),
        #              ((0.375, 0.475), 'T1_S13'), ((0.225, 0.475), 'T1_S13'), ((0.225, 0.825), 'T1_S13'),
        #              ((0.325, 0.825), 'T3_S20'), ((0.325, 0.675), 'T3_S20'), ((0.825, 0.675), 'T3_S20'),
        #              ((0.825, 0.775), 'T3_S20'), ((0.775, 0.775), 'T3_S20'), ((0.775, 0.475), 'T3_S20'),
        #              ((0.825, 0.475), 'T3_S20'), ((0.825, 0.425), 'T3_S20'), ((0.825, 0.425), 'accept_S15'),
        #              ((0.825, 0.425), 'accept_S15'), ((0.825, 0.475), 'T0_S15'), ((0.775, 0.475), 'T0_S15'),
        #              ((0.775, 0.775), 'T0_S15'), ((0.825, 0.775), 'T0_S15'), ((0.825, 0.825), 'T0_S15'),
        #              ((0.825, 0.675), 'T1_S12'), ((0.775, 0.675), 'T1_S12'), ((0.225, 0.675), 'T1_S12'),
        #              ((0.225, 0.825), 'T1_S12'), ((0.325, 0.825), 'T2_S15'), ((0.325, 0.675), 'T2_S15'),
        #              ((0.375, 0.675), 'T2_S15'), ((0.375, 0.425), 'T2_S15'), ((0.425, 0.425), 'T2_S15'),
        #              ((0.475, 0.425), 'T3_S20'), ((0.475, 0.475), 'T3_S20'), ((0.825, 0.475), 'T3_S20'),
        #              ((0.825, 0.425), 'T3_S20'), ((0.825, 0.425), 'accept_S15')]
        # [] (<> e1 && <> e2) && [](<> e3 && <> e4) && <> e5 && <> e6  && [] (e1 -> X (!e1 U e2))' \
        #                    '&& [] (e3 -> X (!e3 U e4))
        # self.path = [((0.025, 0.025), 'T0_init'), ((0.075, 0.025), 'T0_init'), ((0.075, 0.075), 'T0_init'),
        #              ((0.125, 0.075), 'T0_init'), ((0.125, 0.225), 'T0_init'), ((0.125, 0.525), 'T0_S138'),
        #              ((0.225, 0.525), 'T0_S142'), ((0.225, 0.825), 'T0_S142'), ((0.225, 0.675), 'T1_S144'),
        #              ((0.375, 0.675), 'T1_S144'), ((0.775, 0.675), 'T1_S144'), ((0.775, 0.775), 'T1_S144'),
        #              ((0.825, 0.775), 'T1_S144'), ((0.825, 0.825), 'T1_S144'), ((0.825, 0.775), 'T2_S142'),
        #              ((0.775, 0.775), 'T2_S142'), ((0.775, 0.475), 'T2_S142'), ((0.825, 0.475), 'T2_S142'),
        #              ((0.825, 0.425), 'T2_S142'), ((0.825, 0.475), 'T3_S143'), ((0.425, 0.475), 'T3_S143'),
        #              ((0.425, 0.425), 'T3_S143'), ((0.425, 0.425), 'accept_S142'), ((0.425, 0.425), 'accept_S142'),
        #              ((0.425, 0.475), 'T0_S142'), ((0.225, 0.475), 'T0_S142'), ((0.225, 0.825), 'T0_S142'),
        #              ((0.325, 0.825), 'T1_S144'), ((0.325, 0.675), 'T1_S144'), ((0.775, 0.675), 'T1_S144'),
        #              ((0.775, 0.775), 'T1_S144'), ((0.825, 0.775), 'T1_S144'), ((0.825, 0.825), 'T1_S144'),
        #              ((0.825, 0.775), 'T2_S142'), ((0.775, 0.775), 'T2_S142'), ((0.775, 0.475), 'T2_S142'),
        #              ((0.825, 0.475), 'T2_S142'), ((0.825, 0.425), 'T2_S142'), ((0.825, 0.475), 'T3_S143'),
        #              ((0.475, 0.475), 'T3_S143'), ((0.475, 0.425), 'T3_S143'), ((0.425, 0.425), 'T3_S143'),
        #              ((0.425, 0.425), 'accept_S142')]
        # [] (<> e1 && <> e2) && [](<> e3 && <> e4) &&  []( <> e5 && <> e6)  && [] (e1 -> X (!e1 U e2))' \
            #                '&& [] (e3 -> X (!e3 U e4)) && [] (e5 -> X (!e5 U e6))
        self.path = [((0.025, 0.025), 'T0_init'), ((0.075, 0.025), 'T0_init'), ((0.075, 0.075), 'T0_init'),
                     ((0.175, 0.075), 'T0_init'), ((0.175, 0.825), 'T0_init'), ((0.225, 0.825), 'T0_init'),
                     ((0.325, 0.825), 'T1_S203'), ((0.325, 0.675), 'T1_S203'), ((0.775, 0.675), 'T1_S203'),
                     ((0.775, 0.775), 'T1_S203'), ((0.825, 0.775), 'T1_S203'), ((0.825, 0.825), 'T1_S203'),
                     ((0.825, 0.775), 'T2_S1'), ((0.775, 0.775), 'T2_S1'), ((0.775, 0.475), 'T2_S1'),
                     ((0.825, 0.475), 'T2_S1'), ((0.825, 0.425), 'T2_S1'), ((0.825, 0.475), 'T3_S209'),
                     ((0.525, 0.475), 'T3_S209'), ((0.475, 0.475), 'T3_S209'), ((0.475, 0.425), 'T3_S209'),
                     ((0.425, 0.425), 'T3_S209'), ((0.275, 0.425), 'T4_S1'), ((0.275, 0.275), 'T4_S1'),
                     ((0.225, 0.275), 'T4_S1'), ((0.225, 0.225), 'T4_S1'), ((0.125, 0.225), 'T4_S1'),
                     ((0.125, 0.525), 'T5_S216'), ((0.125, 0.525), 'accept_S1'), ((0.125, 0.525), 'accept_S1'),
                     ((0.225, 0.525), 'T0_init'), ((0.225, 0.825), 'T0_init'), ((0.325, 0.825), 'T1_S203'),
                     ((0.325, 0.675), 'T1_S203'), ((0.775, 0.675), 'T1_S203'), ((0.775, 0.775), 'T1_S203'),
                     ((0.825, 0.775), 'T1_S203'), ((0.825, 0.825), 'T1_S203'), ((0.825, 0.775), 'T2_S1'),
                     ((0.775, 0.775), 'T2_S1'), ((0.775, 0.475), 'T2_S1'), ((0.825, 0.475), 'T2_S1'),
                     ((0.825, 0.425), 'T2_S1'), ((0.825, 0.475), 'T3_S209'), ((0.625, 0.475), 'T3_S209'),
                     ((0.475, 0.475), 'T3_S209'), ((0.475, 0.425), 'T3_S209'), ((0.425, 0.425), 'T3_S209'),
                     ((0.375, 0.425), 'T4_S1'), ((0.325, 0.425), 'T4_S1'), ((0.275, 0.425), 'T4_S1'),
                     ((0.275, 0.275), 'T4_S1'), ((0.225, 0.275), 'T4_S1'), ((0.225, 0.225), 'T4_S1'),
                     ((0.125, 0.225), 'T4_S1'), ((0.125, 0.525), 'T5_S216'), ((0.125, 0.525), 'accept_S1')]

        n = 3
        added = []
        for k in range(n):
            while True:
                index = random.randint(0, len(self.path) - 2)
                point = self.path[index][0]
                newpoint1 = (round(point[0] - 0.025, 10), round(point[1] - 0.025, 10))
                point2 = self.path[index + 1][0]
                newpoint2 = (round(point2[0] - 0.025, 10), round(point2[1] - 0.025, 10))
                newpoint = (round_down(random.uniform(newpoint1[0], newpoint2[0]), 1),
                            round_down(random.uniform(newpoint1[1], newpoint2[1]), 1))
                # print(newpoint1, newpoint2, newpoint)
                # space = numpy.floor((numpy.array(newpoint2) - numpy.array(newpoint))/(1/r))
                #
                # newpoint = (newpoint[0] + round(random.randint(0, space[0]) * 1/r, 10) +
                #             newpoint[1] + round(random.randint(0, space[1]) * 1/r), 10)

                # newpoint = (0.05, 0)
                if newpoint in added or (newpoint == point or newpoint == point2):
                    continue
                if newpoint not in center and newpoint not in obs_center:
                    self.obs['o1{0}'.format(4 + k)] = Polygon([(round(newpoint[0], 10), round(newpoint[1], 10)),
                                                               (round(newpoint[0] + mul * r, 10),
                                                                round(newpoint[1], 10)),
                                                               (round(newpoint[0] + mul * r, 10),
                                                                round(newpoint[1] + mul * r, 10)),
                                                               (round(newpoint[0], 10),
                                                                round(newpoint[1] + mul * r, 10))])
                    added.append(newpoint)
                    break
                else:
                    continue

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
            init_state.append((round(r / 2, 10), round(r / 2, 10)))
        self.init_state = (round(r / 2, 10), round(r / 2, 10))
        # self.init_state = (round(r/2+r,10), round(r/2+r,10))

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
            # self.formula = '[]<> (e1 && <> e3) && !e1 U e3'
            # self.formula = '<> e1 && <> e2 && <> e3'


            # self.formula = '[] <> (e1 && <> e2) && []<>( e3 && <> e4) && <> e5  && <> e6 && !e1 U e5'
            # self.formula = '[] <> (e1 && <> e2) && []<>( e3 && <> e4) && [] <> (e5  && <> e6) && !e1 U e5'

            # self.formula = '[]<> e4 && [](<> e3  && <> e1) && (!e1 U e2)  && []!e5'   # 1 2   without any reusable skills, 1 is still much better than 2
            # self.formula = '[]<> e1 && <> e2 && <> e3 && <> e4 && !e3 U e2'
            # test directreuse
            # self.formula = '[]<> e1  && !e1 U e3'
            # self.formula = '[](e1 -> X (!e1 U e5)) && []<>e1 && []<> e5'  # better than [](e1 -> X (!e1 U e5)) && []<>e1
            # self.formula = '[]<> e1 && [] <> e2'  # worse than '[]<> e1 && <> e5'
            # self.formula = '[]<> (e1 && <> (e3 && <> e2))'
            # self.formula = '[] (<> e4 && <> e3)'
            # self.formula = '[]<> e5'
            # self.formula = '[] (<> e1 && <> e2  && <> e3  && <> e4 && <> e5 && <> e6)'
            # self.formula = '[] (<> e1 && <> e2)'
            # self.formula = '[] (<> e1 && <> e4) && [] (e1 -> X (!e1 U e4))'
            # self.formula = '[] (<> e1 && <> e2  && <> e3  && <> e4 && <> e5 && <> e6)'
            # self.formula = '[]<> (e1 && <> e2)  && []<> (e3 && <> e4)  && !e1 U e3'
            # self.formula = '[] (<> e1 && <> e2) && [](<> e3 && <> e4) && <> e5 && <> e6  && [] (e1 -> X (!e1 U e2))' \
            #                '&& [] (e3 -> X (!e3 U e4))'
            self.formula = '[] (<> e1 && <> e2) && [](<> e3 && <> e4) &&  []( <> e5 && <> e6)  && [] (e1 -> X (!e1 U e2))' \
                           '&& [] (e3 -> X (!e3 U e4)) && [] (e5 -> X (!e5 U e6))'

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
               self.formula_comp, self.exclusion, self.num, self.path
