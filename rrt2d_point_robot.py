max_iterations = 300

import random
from matplotlib import pyplot as plt
from PIL import Image
from typing import List
import numpy as np

cherry = (210, 4, 45)
forest_green = (34, 139, 34)
kelly_green = (76, 187, 23)
mauve = (224, 176, 255)


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT2D:
    def __init__(self, x, y):
        self.swath = []
        self.path = []
        self.x_bound = x
        self.y_bound = y

    def sample(self):
        """
        Returns a random node for expansion
        """
        x_sample = int(random.uniform(0, self.x_bound))
        y_sample = int(random.uniform(0, self.y_bound))
        return Node(x_sample, y_sample)

    def find_nearest(self, r_node: Node):
        # def find_nearest(swath: List[Tuple], r_node: Tuple):
        """
        Returns the index of the nearest node in the swath from the given
        randomly sampled one
        """
        return np.argmin(
            [(node.x - r_node.x) ** 2 + (node.y - r_node.y) ** 2 for node in
             self.swath]
        )

    @staticmethod
    def close_to_point(r_node: Node, goal: Node , tol: float = 5) -> bool:
        """
        """
        from math import sqrt
        dist = sqrt((goal.x - r_node.x) ** 2 + (goal.y - r_node.y) ** 2)
        return dist < tol

    @staticmethod
    def get_new_node_at_fixed_distance(random, nearest) -> Node:
        """
        """
        from math import atan2, sin, cos
        dist = 2.5
        theta = atan2(random.y - nearest.y, random.x - nearest.x)
        x_new = nearest.x + dist*cos(theta)
        y_new = nearest.y + dist*sin(theta)
        return Node(int(x_new), int(y_new))


    # @staticmethod
    # def collision_check():
    #     # bot_max_distance_threshold = 5  # Change acc. to vehicle
    #     pass

    def plan(self, start: Node, goal: Node):
        """

        """
        grid = Image.new("RGB", (self.x_bound, self.y_bound), color=(255, 255, 255))
        grid.putpixel((start.x, start.y), cherry)
        grid.putpixel((goal.x, goal.y), forest_green)
        plt.imshow(grid)

        self.swath.append(start)
        for i in range(max_iterations):
            sample = self.sample()
            nearest = self.swath[self.find_nearest(sample)]
            new_node = self.get_new_node_at_fixed_distance(sample, nearest)
            # sample_new = local_planner(nearest, sample)
            # collision = collision_check()
            # if collision check fails, should you change control input or start
            # again with a fresh sample?

            new_node.parent = nearest
            self.swath.append(new_node)

            grid.putpixel((new_node.x, new_node.y), kelly_green)
            plt.imshow(grid)
            plt.plot([nearest.x, new_node.x], [nearest.y, new_node.y], "g", linestyle='-')
            plt.pause(0.001)

            if self.close_to_point(new_node, goal):
                goal.parent = new_node
                self.swath.append(goal)
                break

        # Form the path from swath and display
        current = goal
        while current != start:
            self.path.append(current)
            next_node = current.parent
            plt.plot(
                [current.x, next_node.x], [current.y, next_node.y], color='#ee9090', linestyle='-'
            )
            current = next_node
        plt.show()


    # def just_plan(self, start: Node, goal: Node):
    #     """
    #     """
    #     self.swath.append(start)
    #     for i in range(max_iterations):
    #         sample = self.sample()
    #         nearest = self.swath[self.find_nearest(self.swath, sample)]
    #
    #         sample.parent = nearest
    #         self.swath.append(sample)
    #
    #         if self.close_to_point(sample, goal):
    #             goal.parent = sample
    #             self.swath.append(goal)
    #             break
    #
    #     # Form the path from swath
    #     current = goal
    #     while current != start:
    #         self.path.append(current)
    #         current = current.parent
    #     self.path.reverse()  # Since it is formed in the reverse order


    # def animate(self, start: Node, goal: Node):
    #     """
    #     """
    #     grid = Image.new("RGB", (self.x_bound, self.y_bound), color=(255, 255, 255))
    #     grid.putpixel((start.x, start.y), cherry)
    #     grid.putpixel((goal.x, goal.y), forest_green)
    #     plt.imshow(grid)
    #
    #     swath_len = len(self.swath)
    #     for index, node in enumerate(self.swath):
    #         if index == swath_len - 1:
    #             continue
    #         next_node = self.swath[index + 1]
    #         parent_node = next_node.parent
    #         grid.putpixel((next_node.x, next_node.y), kelly_green)
    #         plt.imshow(grid)
    #         plt.plot([next_node.x, parent_node.x], [next_node.y, parent_node.y], "g", linestyle='-')
    #         # plt.pause(0.001)
    #
    #     # Display the path
    #     for index, node in enumerate(self.path):
    #         next_node = self.swath[index + 1]
    #         plt.plot(
    #             [node.x, next_node.x], [node.y, next_node.y], color='#ee9090', linestyle='-'
    #         )
    #     plt.show()

    @staticmethod
    def local_plan(self, nearest, sample):
        from math import cos, sin, atan2
        dt = 0.01
        lin_vel = 5
        ang_vel = 5

        theta = atan2((sample.y - nearest.y), (sample.x - nearest.x))
        x = nearest.x
        y = nearest.y

        theta += ang_vel * dt
        x += lin_vel * cos(theta) * dt
        y += lin_vel * sin(theta) * dt

        pass


if __name__ == "__main__":
    rrt_planner = RRT2D(100, 100)
    start_node = Node(5, 5)
    goal_node = Node(80, 80)
    rrt_planner.plan(start_node, goal_node)
    # rrt_planner.just_plan(start_node, goal_node)
    # rrt_planner.animate(start_node, goal_node)
