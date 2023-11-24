"""

Grid based Dijkstra planning

author: Atsushi Sakai(@Atsushi_twi)

"""

import matplotlib.pyplot as plt
import math

show_animation = True


class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]    장애물 x좌표
        oy: y position list of Obstacles [m]    장애물 y좌표
        resolution: grid resolution [m]         그리드 해상도
        rr: robot radius[m]                     로봇 반지름
        """

        self.min_x = None                     # 맵의 최소 x좌표
        self.min_y = None                     # 맵의 최대 x좌표
        self.max_x = None                     # 맵의 최소 y좌표
        self.max_y = None                     # 맵의 최대 y좌표
        self.x_width = None                   # 검색되는 맵의 x너비
        self.y_width = None                   # 검색되는 맵의 y너비
        self.obstacle_map = None              # 장애물 좌표 맵, calc_obstacle_map 통해 만들어짐

        self.resolution = resolution          # 해상도, grid_size로 정의
        self.robot_radius = robot_radius      # 로봇의 반지름
        self.calc_obstacle_map(ox, oy)        # ox, oy 기준으로 맵 생성
        self.motion = self.get_motion_model() # 로봇의 이동 방법(앞뒤좌우와 대각선 8방향과 각각의 이동 거리)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        ## round((position - minp) / self.resolution)
        # calc_xy_index >>> 좌표에 해상도, 맵 크기 적용
        # Node : self... x, y, cost, parent_index
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict() # 딕셔너리 생성
        open_set[self.calc_index(start_node)] = start_node # 노드의 x,y값에 따른 index 생성(key값 생성을 위함)

        while True:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)
        # 노드의 x, y가 달라지면 calc_index의 값이 달라짐
        
    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        # ox = [-10~59,  60*70, -10~60, -10*71,  20*50,  40*40]
        # oy = [-10*70, -10~59,  60*71, -10~60, -10~39,  60~21]

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        # x, y 너비 = (최대-최소)/해상도
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]          # 모든 맵을 False로 채우기
        
        for ix in range(self.x_width): # x_width = 70, ix = 0~69
            x = self.calc_position(ix, self.min_x) # x좌표에 해상도와 맵 크기 적용
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y) # y좌표에 해상도와 맵 크기 적용
                for iox, ioy in zip(ox, oy):    # ox와 oy를 (ox1, oy1), (ox2, oy2)...꼴로 변환
                    d = math.hypot(iox - x, ioy - y)    # 로봇의 위치와 장애물과의 거리 차이
                    if d <= self.robot_radius:          # 장애물과 거리 확보 시
                        self.obstacle_map[ix][iy] = True# 현재 위치를 True로 설정
                        break

    @staticmethod
    def get_motion_model():
        # return >> self.motion
        # dx, dy, cost
        # x변화, y변화, 이동 거리
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)], # math.sqrt(2) = 2^(1/2) (재곱근 2)
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = -5.0  # [m]
    sy = -5.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 1.0  # [m] # 검색할 맵의 크기 # 70으로 나누어 떨어지지 않으면 오류 발생
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)            # ox = [-10~59]
        oy.append(-10.0)        # oy = [-10*70]
    for i in range(-10, 60):
        ox.append(60.0)         # ox = [ 60*70]
        oy.append(i)            # oy = [-10~59]
    for i in range(-10, 61):
        ox.append(i)            # ox = [-10~60]
        oy.append(60.0)         # oy = [ 60*71]
    for i in range(-10, 61):
        ox.append(-10.0)        # ox = [-10*71]
        oy.append(i)            # oy = [-10~60]
    for i in range(-10, 40):
        ox.append(20.0)         # ox = [ 20*50]
        oy.append(i)            # oy = [-10~39]
    for i in range(0, 40):
        ox.append(40.0)         # ox = [ 40*40]
        oy.append(60.0 - i)     # oy = [ 60~21]

    # ox = [-10~59,  60*70, -10~60, -10*71,  20*50,  40*40]
    # oy = [-10*70, -10~59,  60*71, -10~60, -10~39,  60~21]
            #아래   #오른쪽    #위    #왼쪽  #장애물1  #장애물2

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")  # .k : 검은 점
        plt.plot(sx, sy, "og")  # og : 초록 원
        plt.plot(gx, gy, "xb")  # xb : 파란 X
        plt.grid(True)          # 격자 표시
        plt.axis("equal")       # 축 비율을 동등하게 설정

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
                              # 시작점,  끝점
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
