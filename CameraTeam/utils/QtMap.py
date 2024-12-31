"""
Contains the RobotMap class that has generates a bitmap for PyQt simulation.
"""
import math

import numpy as np

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor
from PyQt6.QtCore import Qt



COLOR_MAP = {0: QColor("white"), 1: QColor("black"),
             2: QColor("red"), 3: QColor("blue"), 4: QColor("grey")}
DEFAULT_SCALE = 0.5  # Scale from PyBullet to map
CELL_SIZE = 10  # Pixel length/width of each cell
WHITE = 0
BLACK = 1
RED = 2
BLUE = 3
GREY = 4


class RobotMap(QWidget):
    def __init__(self, grid_size):
        super().__init__()
        # Constants
        self.GRID_SIZE = grid_size
        self.OFFSET = grid_size // 4

        self.grid = np.zeros((grid_size, grid_size), dtype=int)
        self.robot_x = 0
        self.robot_y = 0

        self.ray_angles = None
        self.num_rays = None
        self.end_angle = None
        self.start_angle = None
        self.ray_len = None
        self.objectlist = None

        # Set the widget size based on the grid dimensions
        self.setFixedSize(self.GRID_SIZE * CELL_SIZE // 2, self.GRID_SIZE * CELL_SIZE // 2)

    def update_map(self):
        """Update the grid and trigger a repaint."""
        self.update()

    def first_calculate_matrix(self, robot_pos, ray_pos, scaling_factor=DEFAULT_SCALE):
        #  Sets previous robot position to white
        prev_robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        prev_robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[prev_robot_grid_x][prev_robot_grid_y] = WHITE

        # Get new robot position and set color in map
        self.robot_x, self.robot_y, _ = robot_pos
        robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[robot_grid_x][robot_grid_y] = RED

        #  Iterates through ray data and sets position to black
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):
                continue
            grid_x = int(ray_x // scaling_factor) + self.OFFSET
            grid_y = int(ray_y // scaling_factor) + self.OFFSET
            if (grid_x == robot_grid_x and grid_y == robot_grid_y):
                continue
            self.grid[int(grid_x)][int(grid_y)] = BLACK

        self.update_map()

    def bresenham(self, x1, y1, x2, y2):
        """Bresenham's line algorithm for calculating the path between two points on a grid."""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return points

    def is_within_lidar_cone(self, point_x, point_y, robot_x, robot_y,
                             yaw, ray_angles, ray_len,
                             scaling_factor=DEFAULT_SCALE) -> bool:
        """
        Determines if a point is within the LIDAR cone using a list of ray angles.
        
        Args:
            point_x, point_y: The coordinates of the point to check.
            robot_x, robot_y: The robot's position.
            yaw: The robot's current yaw (orientation) in radians.
            ray_angles: List of angles (in radians) that define the LIDAR rays.
            ray_len: The maximum range of the LIDAR rays.
        
        Returns:
            bool: True if the point is within the LIDAR cone, False otherwise.
        """
        for theta in ray_angles:

            adjusted_angle = theta + yaw

            # Convert the adjusted angle to radians
            #angle_rad = math.radians(adjusted_angle)
            angle_rad = adjusted_angle - math.pi/2
            #print(ray_num)
            # Calculate the end coordinates of the ray using ray_len and the adjusted angle
            ray_end_x = self.robot_x + self.ray_len * math.cos(angle_rad)
            ray_end_y = self.robot_y + self.ray_len * math.sin(angle_rad)
            grid_x = int(ray_end_x // scaling_factor) + self.OFFSET
            grid_y = int(ray_end_y // scaling_factor) + self.OFFSET
            ray_path = self.bresenham(robot_x, robot_y, grid_x, grid_y)
            # Mark the path of the ray on the grid
            for (x, y) in ray_path:
                if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                    if (x,y) == (point_x, point_y):
                        return True

        return False

    def second_calculate_matrix(self, robot_pos, ray_pos, gui_values,
                                yaw, rays_data, objectlist, distance=None,
                                scaling_factor=DEFAULT_SCALE) -> list:
        # Reset the previous robot position to GREY
        self.ray_len = gui_values["ray_len"]
        self.start_angle = gui_values["start_angle"]
        self.end_angle = gui_values["end_angle"]
        self.num_rays = gui_values["num_rays"]

        prev_robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        prev_robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[prev_robot_grid_x][prev_robot_grid_y] = WHITE

        # Update the robot's position
        self.robot_x, self.robot_y, _ = robot_pos
        robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[robot_grid_x][robot_grid_y] = RED  # Mark the robot's position

        self.objectlist = objectlist
        #angle_step = (end_angle - start_angle)/(num_rays-1)

        # Mark the ray paths using Bresenham's line algorithm
        ray_num = 0
        ray_ids =[]
        hit_cords =[]
        path_coords =[]
        #print(len(ray_pos))


        a = self.end_angle *(math.pi/180)
        b = (self.start_angle - self.end_angle)*(math.pi/180)
        ray_angles =[]
        for i in range(self.num_rays):
            theta = float(a) + (float(b) * (float(i)/self.num_rays))
            ray_angles.append(theta)
        self.ray_angles = ray_angles
        
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):


                theta = ray_angles[ray_num]
                adjusted_angle = theta + yaw


                # Convert the adjusted angle to radians
                #angle_rad = math.radians(adjusted_angle)
                angle_rad = adjusted_angle - math.pi/2
                #print(ray_num)
                # Calculate the end coordinates of the ray using ray_len and the adjusted angle
                ray_end_x = self.robot_x + self.ray_len * math.cos(angle_rad)
                ray_end_y = self.robot_y + self.ray_len * math.sin(angle_rad)
                grid_x = int(ray_end_x // scaling_factor) + self.OFFSET
                grid_y = int(ray_end_y // scaling_factor) + self.OFFSET
                ray_path = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                # Mark the path of the ray on the grid
                for (x, y) in ray_path:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                        if self.grid[x][y] != RED: # Don't overwrite the robot's position or previous object hit data
                            #self.grid[x][y] = GREY  # Mark the ray path as grey
                            path_coords.append((x,y))
                            continue
            else:
                # Convert ray endpoint to grid coordinates
                # hit_id = ray_num
                grid_x = int(ray_x // scaling_factor) + self.OFFSET
                grid_y = int(ray_y // scaling_factor) + self.OFFSET
                # Perform Bresenham's algorithm to find the path of the ray
                ray_path = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)

                # Mark the path of the ray on the grid
                for (x, y) in ray_path:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                        if self.grid[x][y] != RED:  # Don't overwrite the robot's position or previous object hit data
                            #self.grid[x][y] = GREY  # Mark the ray path as grey
                            path_coords.append((x,y))


                            continue

                # Mark the hit point with BLACK (or other color if needed)
                if 0 <= grid_x < self.GRID_SIZE and 0 <= grid_y < self.GRID_SIZE:
                    #self.grid[grid_x][grid_y] = BLACK
                    if (grid_x, grid_y) not in self.objectlist:
                        self.objectlist.append((grid_x,grid_y))
                    #print(len(self.objectlist))
                    hit_cords.append((grid_x,grid_y))
                    path_coords.append((x,y))

                ray_ids.append(ray_num)
            ray_num = ray_num+1

        to_remove = []
        # count = 0
        for obj_x, obj_y in self.objectlist:
            if (obj_x, obj_y) in path_coords:
                #count = count +1
                #print(count)
                for hits_x, hits_y in hit_cords:
                    if hits_x == obj_x and hits_y == obj_y:
                        self.grid[obj_x][obj_y] = BLACK
                        break
                else:
                    self.grid[obj_x][obj_y] = WHITE
                    to_remove.append((obj_x, obj_y))

        for item in to_remove:
            self.objectlist.remove(item)

        self.update_map()
        return self.objectlist

    def third_calculate_matrix(self, robot_pos, ray_pos, gui_values,
                               yaw, rays_data, distance=None,
                               scaling_factor=DEFAULT_SCALE):
        """
        Update the map grid using Bresenham's algorithm to mark ray paths.
        - Cells along the ray path are set to gray (if no hit occurs).
        - Ray endpoints are set to black (if a hit occurs).
        """
        # Reset the previous robot position to GREY
        self.ray_len = gui_values["ray_len"]
        self.start_angle = gui_values["start_angle"]
        self.end_angle = gui_values["end_angle"]
        self.num_rays = gui_values["num_rays"]
        # hit_dat = rays_data[:, 2]
        # car_heading = robot_pos[2]

        prev_robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        prev_robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[prev_robot_grid_x][prev_robot_grid_y] = GREY

        # Update the robot's position
        self.robot_x, self.robot_y, _ = robot_pos
        robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[robot_grid_x][robot_grid_y] = RED  # Mark the robot's position

        #angle_step = (end_angle - start_angle)/(num_rays-1)


        # Mark the ray paths using Bresenham's line algorithm
        ray_num = 0
        ray_ids =[]
        #print(len(ray_pos))

        a = self.end_angle *(math.pi/180)
        b = (self.start_angle - self.end_angle)*(math.pi/180)
        ray_angles =[]
        # ray_from, ray_to = [], []
        for i in range(self.num_rays):
            theta = float(a) + (float(b) * (float(i)/self.num_rays))

            ray_angles.append(theta)

        self.ray_angles = ray_angles

        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):

                theta = ray_angles[ray_num]
                adjusted_angle = theta + yaw

                # Convert the adjusted angle to radians
                #angle_rad = math.radians(adjusted_angle)
                angle_rad = adjusted_angle - math.pi/2
                #print(ray_num)
                # Calculate the end coordinates of the ray using ray_len and the adjusted angle
                ray_end_x = self.robot_x + self.ray_len * math.cos(angle_rad)
                ray_end_y = self.robot_y + self.ray_len * math.sin(angle_rad)
                grid_x = int(ray_end_x // scaling_factor) + self.OFFSET
                grid_y = int(ray_end_y // scaling_factor) + self.OFFSET
                ray_path = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                # Mark the path of the ray on the grid
                for (x, y) in ray_path:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                        # Don't overwrite the robot's position or previous object hit data
                        if self.grid[x][y] != RED and self.grid[x][y] != BLACK:
                            self.grid[x][y] = GREY  # Mark the ray path as grey
                            #print(f"Ray {ray_num}: angle={angle_rad}, ray_end_x={ray_end_x}, ray_end_y={ray_end_y}")

                            # If the ray is directly in front, color it blue
               # if ray_num == round((num_rays // 2)):  # Assuming front ray is the middle one
                    #self.grid[grid_x][grid_y] = BLUE  # Mark the ray directly in front as blue """

            else:
                # Convert ray endpoint to grid coordinates
                grid_x = int(ray_x // scaling_factor) + self.OFFSET
                grid_y = int(ray_y // scaling_factor) + self.OFFSET
                # Perform Bresenham's algorithm to find the path of the ray
                ray_path = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)

                # Mark the path of the ray on the grid
                for (x, y) in ray_path:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                        # Don't overwrite the robot's position or previous object hit data
                        if self.grid[x][y] != RED and self.grid[x][y] != BLACK:
                            self.grid[x][y] = GREY  # Mark the ray path as grey

                # Mark the hit point with BLACK (or other color if needed)
                if 0 <= grid_x < self.GRID_SIZE and 0 <= grid_y < self.GRID_SIZE:
                    self.grid[grid_x][grid_y] = BLACK
                ray_ids.append(ray_num)
            ray_num = ray_num+1

        self.update_map()

    def paintEvent(self, event):
        """
        Default method for initializing a QPainter OGM
        and filling it based off of self.grid.
        """
        painter = QPainter(self)
        for row in range(self.GRID_SIZE):
            for col in range(self.GRID_SIZE):
                # Set the brush color based on the matrix value
                painter.setBrush(COLOR_MAP[self.grid[row][col]])
                painter.setPen(Qt.PenStyle.NoPen)  # Optional: remove grid lines
                # Draw the cell rectangle
                painter.drawRect(
                    col * CELL_SIZE,
                    row * CELL_SIZE,
                    CELL_SIZE,
                    CELL_SIZE,
                )

    def reset_map(self):
        """
        Resets the OGM to all white space.
        """
        self.grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)
        self.update_map()
