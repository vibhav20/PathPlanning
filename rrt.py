import matplotlib.pyplot as plt
import math
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString


def RRT(start, goal, obstacle_list):

    polygon = Polygon(obstacle_list[0])
    polygon1 = Polygon(obstacle_list[1])
    polygon2 = Polygon(obstacle_list[2])

    def distance(x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 1 / 2

    def insidepolygon(a, b):
        point = Point(a, b)
        return polygon.contains(point)

    def insidepolygon1(a, b):
        point1 = Point(a, b)
        return polygon1.contains(point1)

    def insidepolygon2(a, b):
        point2 = Point(a, b)
        return polygon2.contains(point2)

    def intersect(x1, y1, x2, y2):
        c = 0
        line = LineString([(x1, y1), (x2, y2)])
        for i in range(len(obstacle_list)):
            for j in range(len(obstacle_list[i]) - 1):
                line1 = LineString([obstacle_list[i][j], obstacle_list[i][j + 1]])
                # if line from parent to node intersects any line of obstacle
                if line.intersects(line1):
                    c = c + 1
                if (j + 2) == len(obstacle_list):
                    line2 = LineString([obstacle_list[i][j + 1], obstacle_list[i][0]])
                    if line.intersects(line2):
                        c = c + 1
        if c == 0:  # no intersection found
            return False
        else:
            return True

    def trackback(m):  # tracksback  from goal to start
        while m != 0:
            plt.plot(
                [xtree[m], xparent[m - 1]],
                [ytree[m], yparent[m - 1]],
                "b.-",
                linewidth=0.3,
                markersize=3.5,
            )
            m = xtree.index(xparent[m - 1])

    xs = 1
    ys = 1
    xtree = [start[0]]
    ytree = [start[1]]
    xparent = []
    yparent = []
    path = []
    n = 0  #  keeps track of iterations

    while n < 1000:  # number of iterations algorithm should run for
        minimum = 100
        j = 0
        k = 0  # stores position of sampled point which is closest to goal
        x = random.uniform(0, 10)
        y = random.uniform(0, 10)
        if (
            insidepolygon(x, y) or insidepolygon1(x, y) or insidepolygon2(x, y)
        ):  # checks if sampled point is inside poly
            continue
        if (distance(x, y, xs, ys)) < 0.5:  # maxdistance=0.5 from last sampled node
            for i in range(0, n + 1):
                dis = distance(xtree[i], ytree[i], x, y)
                if dis <= minimum:
                    minimum = dis
                    j = i  # position of node at minimum distance from x,y
            if intersect(x, y, xtree[j], ytree[j]):  # if line intersects obstacle
                continue
            xtree.append(x)
            ytree.append(y)
            xparent.append(xtree[j])  # keeps track of parent of the added node
            yparent.append(ytree[j])

            plt.plot(
                [x, xparent[n]], [y, yparent[n]], "r.-", linewidth=0.25, markersize=3.5
            )
            if distance(x, y, goal[0], goal[1]) < 0.5:  # goal region
                plt.title("Goal Reached!")
                trackback(n + 1)
                break

            minimum = 100
            for i in range(0, n + 2):  # to find position of point closest to goal
                dis = distance(xtree[i], ytree[i], goal[0], goal[1])
                if dis <= minimum:
                    minimum = dis
                    k = i
            if n % 5 == 0:  # for every fifth iteration 'xs' is set to the point in tree
                xs = xtree[k]  # which is currently closest to goal
                ys = ytree[k]  # so as to steer the path towards goal
            else:
                xs = x
                ys = y
            n = n + 1
    path = zip(xtree, ytree)
    return path


def visualize(path, obstacle_list):

    for i in range(len(obstacle_list)):
        polygon = Polygon(obstacle_list[i])
        plt.plot(*polygon.exterior.xy)

    plt.show()