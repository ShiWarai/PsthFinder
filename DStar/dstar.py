"""

D* grid planning

author: Nirnay Roy

See Wikipedia article (https://en.wikipedia.org/wiki/D*)

"""
import math

from sys import maxsize

import matplotlib.pyplot as plt
import numpy as np
import json
show_animation = True


class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.plot(rx, ry, "-r")
                plt.pause(0.01)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break


def main():
    m = Map(100, 100)
    ox, oy = [], []
     # set obstacle positions
    arrr = []
    with open('array.txt', 'r') as f:
        arrr = json.load(f)
    print(arrr)

    #рассчитать уравнение прямой между двумя точками
    def calc_line_equation(p1, p2):
        if p1[0] == p2[0]:
            m = float("inf")
            c = p1[0]
        else:
            m = (p1[1] - p2[1]) / (p1[0] - p2[0])
            c = p1[1] - m * p1[0]
        return [m, c]

    #для всех точек из массива arrr получить уравнение прямой между двумя точками
    def get_line_equation(arrr):
        line_equation = []
        for i in range(len(arrr) - 1):
            line_equation.append(calc_line_equation(arrr[i], arrr[i + 1]))
        return line_equation

    arr_MC = get_line_equation(arrr)
    #для каждой пары точек из arrr сгенерировать точки между ними с шагом в 0.1
    def get_points_between_two_points(p1, p2):
        points = []
        m, c = calc_line_equation(p1, p2)
        if m == float("inf"):
            for i in range(p1[0], p2[0], 0.1):
                points.append([i, p1[1]])
        else:
            for i in np.arange(p1[0], p2[0], 0.1):
                points.append([i, m * i + c])
        print(points)
        return points



    #построить график для всех точек из массива arrr
    def draw_graph(arrr):
        points = []
        for i in range(len(arrr) - 1):
            points += get_points_between_two_points(arrr[i], arrr[i + 1])
            plt.plot()
        # print("**********************")
        # plt.plot([x[0] for x in arrr], [x[1] for x in arrr], 'go')
        # plt.plot([x[0] for x in points], [x[1] for x in points], 'o')
        # plt.show()
        return points
    draw_graph(arrr)

    arrr.extend(draw_graph(arrr))

    ox = [x[0] for x in arrr]
    oy = [x[1] for x in arrr]
    print([(i, j) for i, j in zip(ox, oy)])
    m.set_obstacle([(i, j) for i, j in zip(ox, oy)])

    start = [10, 10]
    goal = [50, 50]
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")

    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
