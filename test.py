# import matplotlib.pyplot as plt
# import numpy as np
#
# x = np.linspace(1, 21, 200)
# y = np.exp(-x)
#
# fig, ax = plt.subplots()
# ax.plot(x, y)
# ax.hlines(y=0.2, xmin=x[0], xmax=x[-1], color='k')
#
# plt.show()

from Problem import problemFormulation
from Visualization import region_plot
import matplotlib.pyplot as plt


workspace, regions, centers, obs, init_state, uni_cost, formula,\
            formula_comp, exclusion, num = problemFormulation(0).Formulation()
ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}


ax = plt.figure(1).gca()
region_plot(regions, 'region', ax, num)
region_plot(obs, 'obs', ax, num)
plt.grid(False)
plt.show()
