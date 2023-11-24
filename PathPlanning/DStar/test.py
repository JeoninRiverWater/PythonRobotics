"""

D* grid planning

author: Nirnay Roy

See Wikipedia article (https://en.wikipedia.org/wiki/D*)

"""
import math

from sys import maxsize

import matplotlib.pyplot as plt

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

    def __init__(self, row, col): # Map(100, 100) from main()
        self.row = row
        self.col = col
        self.map = self.init_map() # 100x100의 배열, 좌표를 나타내는 State가 총 10000개 포함

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
            if x < 0 or x >= self.row or y < 0 or y >= self.col: # x와 y의 범위가 Map의 크기(100,100)을 벗어나면 건너뛰기
                continue

            self.map[x][y].set_state("#")


class Dstar: # dstar = Dstar(m) | m = Map(100, 100)| 장애물, 시작점, 끝점 포함된 데이터
    def __init__(self, maps):
        self.map = maps
        self.open_list = set() # python set(집합)형
        """
        Python set(집합)형
        {x1,x2,x3...}와 같이 사용
        단일 종류의 자료형으로 만들어짐
        중복된 원소를 갖지 않음
        """

    def process_state(self):
        x = self.min_state()
        """open_list에서 state(점 좌표).k 가 가장 작은 것"""
        #>>> state 자체를 가지고 옴

        if x is None: # 예외 처리
            return -1

        k_old = x.k
        
        self.remove(x)

        if k_old < x.h: # k<h?
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
        if not self.open_list: # open_list가 비어있는 경우에 대한 예외처리
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        # open_list에서 state(점 좌표).k 가 가장 작은 것
        return min_state

    def insert(self, state, h_new): # self.insert(end, 0.0)(171)
        # state.t = "new"(기본) (State 클래스의 self.t 참고)
        # state.k = 0    (기본) (State 클래스의 self.k 참고)
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

    def run(self, start, end): # rx, ry = dstar.run(start, end), line 248
                # 시작점, 끝점
        rx = []
        ry = []

        self.insert(end, 0.0)
      # >>> end.t = "open", end.k = 0.0, end.h = 0.0

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
    for i in range(-10, 60): # 기본 사각형 생성(밑변)
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60): # 기본 사각형 생성(오른쪽 변)
        ox.append(60)
        oy.append(i)
    for i in range(-10, 61): # 기본 사각형 생성(윗변)
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61): # 기본 사각형 생성(왼쪽 변)
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 40): # 벽 생성(왼쪽 벽)
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):   # 벽 생성(오른쪽 벽)
        ox.append(40)
        oy.append(60 - i)
        
    # for i in range(len(ox)) : 
    #     print(f"({ox[i]}, {oy[i]})", end=" ") # 생성된 벽 좌표 출력
    # print([(i, j) for i, j in zip(ox, oy)])     # 벽 좌표를 [(x1, y1), (x2, y2)...]의 형식으로 변환
    m.set_obstacle([(i, j) for i, j in zip(ox, oy)]) # 생성된 좌표 토대로 기본 장애물 생성

    start = [10, 10]
    goal = [50, 50]
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal") # 표시되는 데이터의 x, y 단위를 같게 함
        """
        plt.plot(x, y, marker_style)
        marker_style[0] = 타입 설정('.', '*', '+', 'o', 'x' 등등...)
        marker_style[1] = 색상 설정(k, b, g 등등...)
        """

    start = m.map[start[0]][start[1]] # start(시작점)을 (10, 10)으로 지정
    end = m.map[goal[0]][goal[1]]     # end(끝점)을 (50, 50)으로 지정
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()