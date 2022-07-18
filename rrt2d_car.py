max_iterations = 300

import random
from math import sqrt, sin, cos, tan, radians, atan2
from typing import List

import numpy as np
from PIL import Image
from matplotlib import pyplot as plt

from control import move_to_pose

# cherry = (210, 4, 45)
cherry = (171,35,40)
forest_green = (34, 139, 34)
kelly_green = (76, 187, 23)

mauve = (224, 176, 255)
pink = (238, 144, 144)
road = (128,128,128)


class Node:
    def __init__(self, x, y, theta: float = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = None


class RRT2D:
    def __init__(self, x, y):
        self.swath = []
        self.path = []
        self.x_bound = x
        self.y_bound = y
        self.chassis_len = 2  # Given: 2.8m
        self.grid = None

        # controls inputs
        self.steering_angles = [-60, -45, -30, 0, 30, 45, 60]
        self.vel_for = 15
        self.vel_rev = -15
        self.del_t = 0.7

    def sample(self):
        """
        Returns a random node for expansion
        """
        x_sample = int(random.uniform(0, self.x_bound))
        y_sample = int(random.uniform(0, self.y_bound))
        return Node(x_sample, y_sample)


    def get_child_nodes(self, node_n):
        """
        """
        open_nodes = {}
        for vel in [self.vel_for, self.vel_rev]:
            for angle in self.steering_angles:
                node_heading = node_n.theta + vel * tan(radians(angle)) / self.chassis_len * self.del_t
                node_x = int((node_n.x + vel * cos(radians(node_heading)) * self.del_t))
                node_y = int((node_n.y + vel * sin(radians(node_heading)) * self.del_t))
                node = Node(node_x, node_y, node_heading)
                if self.is_out_of_bounds(node):
                    continue
                open_nodes[node] = {"vel": vel, "steer": angle}
        return open_nodes

    def find_nearest_in_swath(self, r_node: Node) -> Node:
        """
        Returns the index of the nearest node in the swath from the given
        randomly sampled one
        """
        nearest = np.argmin(
            [(node.x - r_node.x) ** 2 + (node.y - r_node.y) ** 2 for node in self.swath]
        )
        return self.swath[nearest]

    def get_new_node(self, nearest: Node, r_node: Node) -> Node:
        """
        """
        potential_nodes = self.get_child_nodes(nearest)  # dict
        potential_nodes = [key for key in potential_nodes.keys()]  # list
        idx = np.argmin(
            [(node.x - r_node.x) ** 2 + (node.y - r_node.y) ** 2 for node in potential_nodes]
        )
        return potential_nodes[idx]

    @staticmethod
    def close_to_point(r_node: Node, goal: Node , tol: float = 10) -> bool:
        """
        """
        dist = sqrt((goal.x - r_node.x) ** 2 + (goal.y - r_node.y) ** 2)
        return dist < tol

    def is_out_of_bounds(self, node: Node):
        return (
            node.x < 0
            or node.x >= self.x_bound
            or node.y < 0
            or node.y >= self.y_bound
        )

    def plan(self, start: Node, goal: Node) -> (List[Node], Image):
        """

        """
        grid = Image.new("RGB", (self.x_bound, self.y_bound), color=road)
        grid.putpixel((start.x, start.y), cherry)
        grid.putpixel((goal.x, goal.y), forest_green)
        # Add divider of road.
        for x_coord in range(self.x_bound):
            grid.putpixel((x_coord , int(self.y_bound/2)), (255, 255, 255))
        # Add static obstacle
        for x_coord in range(40, 60):
            for y_coord in range(37, 47):
                grid.putpixel((x_coord, y_coord), (0, 0, 0))
        # plt.gca().add_patch(
        #     Rectangle((40, 35), 20, 10, linewidth=1, facecolor='#000000')
        # )
        plt.imshow(grid)

        self.swath.append(start)
        for i in range(max_iterations):
            sample = self.sample()
            nearest = self.find_nearest_in_swath(sample)
            new_node = self.get_new_node(nearest, sample)

            if grid.getpixel((new_node.x, new_node.y)) == (0, 0, 0):
                continue

            new_node.parent = nearest
            self.swath.append(new_node)

            grid.putpixel((new_node.x, new_node.y), pink)
            plt.imshow(grid)
            plt.plot([nearest.x, new_node.x], [nearest.y, new_node.y], "y", linestyle='-')
            plt.pause(0.001)

            if self.close_to_point(new_node, goal):
                print("Found the path!")
                goal.parent = new_node
                self.swath.append(goal)
                break

        # Form the path from swath and display
        current = goal
        while current != start:
            self.path.append(current)
            next_node = current.parent
            plt.plot(
                [current.x, next_node.x], [current.y, next_node.y], color='#4CBB17', linestyle='-'
            )
            current = next_node
        self.path.append(start)
        plt.show()
        self.path.reverse()
        return self.path, grid


if __name__ == "__main__":
    rrt_planner = RRT2D(100, 50)
    start = Node(5, 40, 0)
    goal = Node(85, 35, 0)
    path, grid = rrt_planner.plan(start, goal)

    grid = Image.new("RGB", (rrt_planner.x_bound, rrt_planner.y_bound), color=road)
    grid.putpixel((start.x, start.y), cherry)
    grid.putpixel((goal.x, goal.y), kelly_green)

    # Add divider of road.
    for x_coord in range(rrt_planner.x_bound):
        grid.putpixel((x_coord, int(rrt_planner.y_bound / 2)), (255, 255, 255))
    # Add static obstacle
    for x_coord in range(40, 60):
        for y_coord in range(37, 47):
            grid.putpixel((x_coord, y_coord), (0, 0, 0))
    # plt.gca().add_patch(
    #     Rectangle((40, 37.5), 20, 10, linewidth=1, facecolor='#000000')
    # )

    for index, node in enumerate(path):
        if index == len(path) - 1:
            continue
        next_n = path[index + 1]
        next_n.theta = atan2(next_n.y - node.y, next_n.x - node.x)
        plt.plot([next_n.x, node.x], [next_n.y, node.y], "#4CBB17", linestyle='-')

    for index, node in enumerate(path):
        if index == len(path) - 1:
            continue
        next_n = path[index + 1]
        move_to_pose(node.x, node.y, node.theta, next_n.x, next_n.y, next_n.theta, grid)

