import numpy as np
import math
import matplotlib.pyplot as plt
from functools import reduce
from copy import copy

"""
A class to handle the simulation and plotting of state space systems.
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

# Find the difference between two arrays of vectors
def _diff(vector_hist):
    return [(np.zeros(vector_hist[0].shape) if i == 0 else vector_hist[i] - vector_hist[i - 1]) for i in range(len(vector_hist))]

# Find discontinuities in an array of vectors
def _find_discontinuities(vector_hist, thresh = 2):
    values = []
    for i, d in enumerate(_diff(vector_hist)):
        if np.linalg.norm(d) > thresh:
            values.append(i)
    return values

# Plot an array of values without drawing a line at discontinuous points
def _plot_with_discontinuities(ax, x, y, name, i):
    t_copy = [t for t in np.nditer(x)]
    y_copy = [v[i, 0] for v in y]
    disc_points = _find_discontinuities(y_copy)

    for c, dp in enumerate(disc_points):
        y_copy.insert(dp + c, np.nan)
        t_copy.insert(dp + c, np.nan)

    ax.plot(t_copy, y_copy, label = name)

class StateSpaceScenario(object):
    def __init__(self, sys, x_initial, controller, observer, x_hat_initial, name = '', noise = None):
        self.sys = sys

        self.x_initial = x_initial
        self.x_hat_initial = x_hat_initial

        self.controller = controller
        self.observer = observer

        self.reset()

    def reset(self):
        self.sys.x = self.x_initial
        self.observer.x_hat = self.x_hat_initial

        self.x_history = []
        self.x_hat_history = []
        self.r_history = []
        self.u_history = []
        self.t_history = []

    def run(self, goal, total_time):
        self.reset()
        self.t_history = np.linspace(0, total_time, total_time / self.sys.get_current_gains().dt + 1)

        self.controller.r = goal(0)

        for t in self.t_history:
            # Log the variables at n+1
            self.x_history.append(self.sys.x)
            self.x_hat_history.append(self.observer.x_hat)
            self.r_history.append(self.controller.r)
            self.u_history.append(self.sys.u)

            # Order: Get u(n+1) from the controller
            #        Use u(n+1) and y(n) = Cx(n) for the observer
            #        Use u(n+1) to update the actual system
            u = self.controller.update(self.observer.x_hat, goal(t + self.sys.get_current_gains().dt))
            self.observer.update(self.sys.y, u)
            self.sys.update(u)

        self.display()


    def display(self):
        fig = plt.figure()

        num_x_plots = self.sys.x.shape[0]
        num_u_plots = self.sys.u.shape[0]

        # Metric for determining how many plots to make horizontally and vertically
        plots_x = math.floor(math.sqrt(num_x_plots + num_u_plots))
        plots_y = math.ceil((num_x_plots + num_u_plots) / plots_x)

        for i in range(num_x_plots):
            ax = fig.add_subplot(plots_y, plots_x, i + 1)

            # TODO(Kyle) Insert np.nan where there are discontinuities
            for name in ['x', 'x_hat', 'r']:
                hist = getattr(self, name + '_history')
                _plot_with_discontinuities(ax, self.t_history, hist, '{}[{}]'.format(name, i), i)

            ax.legend()

            ax.set_xlabel('t')
            ax.set_ylabel('x[%i]' % i)

        for i in range(num_u_plots):
            ax = fig.add_subplot(plots_y, plots_x, num_x_plots + i + 1)

            _plot_with_discontinuities(ax, self.t_history, self.u_history, 'u[{}]'.format(i), i)

            ax.legend()

            ax.set_xlabel('t')
            ax.set_ylabel('u[%i]' % i)

        plt.tight_layout()

        plt.show()
