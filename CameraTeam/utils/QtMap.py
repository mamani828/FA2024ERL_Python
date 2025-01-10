"""
Contains the RobotMap class that has generates a bitmap for PyQt simulation.
"""
import math

import numpy as np

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QPixmap
from PyQt6.QtCore import Qt



COLOR_MAP = {0: QColor("white"), 1: QColor("black"),
             2: QColor("red"), 3: QColor("blue"), 4: QColor("grey")}
DEFAULT_SCALE = 0.1  # Scale from PyBullet to map
CELL_SIZE = 5  # Pixel length/width of each cell
WHITE = 0
BLACK = 1
RED = 2
BLUE = 3
GREY = 4

#Probabilistic OGM assumptions
PRIOR_OCC = 0.2
SENSOR_OCC = 0.7
SENSOR_EMPTY = 0.7


class RobotMap(QWidget):
    def __init__(self, grid_size):
        super().__init__()
        # Constants
        self.GRID_SIZE = grid_size
        self.OFFSET = grid_size // 4

        self.grid = np.zeros((grid_size, grid_size), dtype=int)
        self.grid_p = np.ones((grid_size, grid_size), dtype=int) * 0.5
        self.robot_x = 0
        self.robot_y = 0

        self.ray_angles = None
        self.num_rays = None
        self.end_angle = None
        self.start_angle = None
        self.ray_len = None
        self.objectlist = None
        self.prob = 0
        self.changed_cells = set()
        # Set the widget size based on the grid dimensions
        self.setFixedSize(self.GRID_SIZE * CELL_SIZE // 2, self.GRID_SIZE * CELL_SIZE // 2)

        # Create QPixmap for an off-screen buffer
        self.pixmap = QPixmap(self.GRID_SIZE * CELL_SIZE // 2, self.GRID_SIZE * CELL_SIZE // 2)
        self.pixmap.fill(QColor('gray')) 
        self.initial = 1 

    def update_map(self):
        """Update the grid and trigger a repaint."""
        self.update()

    def first_calculate_matrix(self, robot_pos, ray_pos, scaling_factor=DEFAULT_SCALE):
        self.prob = 0
        if self.initial == 1: 
            self.pixmap.fill(QColor('white')) #intilize with gray background
            self.initial = 0
        self.prob = 0

        robot_grid_x, robot_grid_y = self.compensate_movement(scaling_factor, robot_pos)

        #  Iterates through ray data and sets position to black
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):
                continue
            grid_x, grid_y = self.to_grid(scaling_factor, ray_x, ray_y)
            if (grid_x == robot_grid_x and grid_y == robot_grid_y):
                continue
            #self.grid[int(grid_x)][int(grid_y)] = BLACK
            self.update_grid(int(grid_x), int(grid_y), BLACK)

        self.update_cell()

    def second_calculate_matrix(self, robot_pos, ray_pos, gui_values,
                                yaw, rays_data, objectlist, distance=None,
                                scaling_factor=DEFAULT_SCALE) -> list:
        if self.initial == 1: 
            self.pixmap.fill(QColor('white')) #intilize with gray background
            self.initial = 0
        self.prob = 0
        robot_grid_x, robot_grid_y = self.compensate_movement(scaling_factor, robot_pos)

        self.objectlist = objectlist
        #angle_step = (end_angle - start_angle)/(num_rays-1)

        # Mark the ray paths using Bresenham's line algorithm
        ray_num = 0
        ray_ids =[]
        hit_cords =[]
        path_coords =[]

        self.get_angles(gui_values)
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):

                theta = self.ray_angles[ray_num]
                adjusted_angle = theta + yaw

                # Convert the adjusted angle to radians
                #angle_rad = math.radians(adjusted_angle)
                angle_rad = adjusted_angle
                #print(ray_num)
                # Calculate the end coordinates of the ray using ray_len and the adjusted angle
                ray_end_x = self.robot_x + self.ray_len * math.cos(angle_rad)
                ray_end_y = self.robot_y + self.ray_len * math.sin(angle_rad)
                grid_x, grid_y = self.to_grid(scaling_factor, ray_end_x, ray_end_y)
                ray_path = RobotMap.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
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
                grid_x, grid_y = self.to_grid(scaling_factor, ray_x, ray_y)
                ray_path = RobotMap.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)

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
                        #self.grid[obj_x][obj_y] = BLACK
                        self.update_grid(obj_x, obj_y, BLACK)
                        
                        break
                else:
                    #self.grid[obj_x][obj_y] = WHITE
                    self.update_grid(obj_x, obj_y, WHITE)
                    to_remove.append((obj_x, obj_y))

        for item in to_remove:
            self.objectlist.remove(item)

        self.update_cell()
        return self.objectlist

    def third_calculate_matrix(self, robot_pos, ray_pos, gui_values,
                               yaw, rays_data, distance=None,
                               scaling_factor=DEFAULT_SCALE):
        """
        Update the map grid using Bresenham's algorithm to mark ray paths.
        - Cells along the ray path are set to gray (if no hit occurs).
        - Ray endpoints are set to black (if a hit occurs).
        """
        if self.initial == 1: 
            self.pixmap.fill(QColor('white')) #intilize with gray background
            self.initial = 0
        self.prob = 0

        robot_grid_x, robot_grid_y = self.compensate_movement_1(scaling_factor, robot_pos)

        #angle_step = (end_angle - start_angle)/(num_rays-1)
        ray_num = 0
        ray_ids =[]
        #print(len(ray_pos))
        self.get_angles(gui_values)
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):

                theta = self.ray_angles[ray_num]
                adjusted_angle = theta + yaw

                # Convert the adjusted angle to radians
                #angle_rad = math.radians(adjusted_angle)
                angle_rad = adjusted_angle
                #print(ray_num)
                # Calculate the end coordinates of the ray using ray_len and the adjusted angle
                ray_end_x = self.robot_x + self.ray_len * math.cos(angle_rad)
                ray_end_y = self.robot_y + self.ray_len * math.sin(angle_rad)
                grid_x, grid_y = self.to_grid(scaling_factor, ray_end_x, ray_end_y)
                ray_path = RobotMap.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                # Mark the path of the ray on the grid
                for (x, y) in ray_path:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                        # Don't overwrite the robot's position or previous object hit data
                        if self.grid[x][y] != RED and self.grid[x][y] != BLACK:
                            #self.grid[x][y] = GREY  # Mark the ray path as grey
                            self.update_grid(x, y, GREY)
                            #print(f"Ray {ray_num}: angle={angle_rad}, ray_end_x={ray_end_x}, ray_end_y={ray_end_y}")

                            # If the ray is directly in front, color it blue
               # if ray_num == round((num_rays // 2)):  # Assuming front ray is the middle one
                    #self.grid[grid_x][grid_y] = BLUE  # Mark the ray directly in front as blue """

            else:
                # Convert ray endpoint to grid coordinates
                grid_x = int(ray_x // scaling_factor) + self.OFFSET
                grid_y = int(ray_y // scaling_factor) + self.OFFSET
                # Perform Bresenham's algorithm to find the path of the ray
                ray_path = RobotMap.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)

                # Mark the path of the ray on the grid
                for (x, y) in ray_path:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                        # Don't overwrite the robot's position or previous object hit data
                        if self.grid[x][y] != RED and self.grid[x][y] != BLACK:
                            #self.grid[x][y] = GREY  # Mark the ray path as grey
                            self.update_grid(x, y, GREY)


                # Mark the hit point with BLACK (or other color if needed)
                if 0 <= grid_x < self.GRID_SIZE and 0 <= grid_y < self.GRID_SIZE:
                    #self.grid[grid_x][grid_y] = BLACK
                    self.update_grid(grid_x, grid_y, BLACK)
                ray_ids.append(ray_num)
            ray_num = ray_num+1

        self.update_cell()

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
            grid_x, grid_y = self.to_grid(scaling_factor, ray_end_x, ray_end_y)
            ray_path = RobotMap.bresenham(robot_x, robot_y, grid_x, grid_y)
            # Mark the path of the ray on the grid
            for (x, y) in ray_path:
                if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                    if (x,y) == (point_x, point_y):
                        return True

        return False

    def get_angles(self, gui_values):
        self.ray_len = gui_values["ray_len"]
        self.start_angle = gui_values["start_angle"]
        self.end_angle = gui_values["end_angle"]
        self.num_rays = gui_values["num_rays"]

        

        #angle_step = (end_angle - start_angle)/(num_rays-1)
        
        #print(len(ray_pos))

        a = self.start_angle *(math.pi/180)
        b = (self.start_angle - self.end_angle)*(math.pi/180)
        ray_angles =[]
        # ray_from, ray_to = [], []
        for i in range(self.num_rays):
            theta = float(a) + (float(b) * (float(i)/self.num_rays))

            ray_angles.append(theta)

        self.ray_angles = ray_angles



    def fourth_calculate_matrix(self, robot_pos, ray_pos, gui_values,
                               yaw, rays_data, distance=None,
                               scaling_factor=DEFAULT_SCALE):
        """
        Implement probabilistic OGM using Arbitrary values to simulate noise i.e 30 percent change that an empty reading is wrong

        """
        # Reset the previous robot position to GREY
        #self.pixmap.fill(QColor('gray')) 
        if self.initial == 1: 
            self.pixmap.fill(QColor('gray')) #intilize with gray background
            self.initial = 0
        self.prob = 1
        self.ray_len = gui_values["ray_len"]
        self.start_angle = gui_values["start_angle"]
        self.end_angle = gui_values["end_angle"]
        self.num_rays = gui_values["num_rays"]
        # hit_dat = rays_data[:, 2]
        # car_heading = robot_pos[2]

        robot_grid_x, robot_grid_y = self.compensate_movement(scaling_factor, robot_pos)

        self.get_angles(gui_values)
    
        ray_num = 0
        ray_ids =[]
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):

                theta = self.ray_angles[ray_num]
                adjusted_angle = theta + yaw
                #print(yaw)

                # Convert the adjusted angle to radians
                #angle_rad = math.radians(adjusted_angle)
                angle_rad = adjusted_angle 
                #print(ray_num)
                # Calculate the end coordinates of the ray using ray_len and the adjusted angle
                ray_end_x = self.robot_x + self.ray_len * math.cos(angle_rad)
                ray_end_y = self.robot_y + self.ray_len * math.sin(angle_rad)
                grid_x, grid_y = self.to_grid(scaling_factor, ray_end_x, ray_end_y)
                ray_path = RobotMap.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
                # Mark the path of the ray on the grid
                for (x, y) in ray_path:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:

                            #update for empty space detection

                        val = self.probabilistic_update(PRIOR_OCC, SENSOR_OCC, SENSOR_EMPTY, self.grid_p[x][y], 0)
                        #self.grid_p[x][y] = self.probabilistic_update(PRIOR_OCC, SENSOR_OCC, SENSOR_EMPTY, self.grid_p[x][y], 0)
                        self.update_grid(x,y,val)
            else:
                # Convert ray endpoint to grid coordinates
                grid_x = int(ray_x // scaling_factor) + self.OFFSET
                grid_y = int(ray_y // scaling_factor) + self.OFFSET
                # Perform Bresenham's algorithm to find the path of the ray
                ray_path = RobotMap.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)

                # Mark the path of the ray on the grid
                for (x, y) in ray_path[0:-1]:
                    if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                        # Don't overwrite the robot's position or previous object hit data
                        val = self.probabilistic_update(PRIOR_OCC, SENSOR_OCC, SENSOR_EMPTY, self.grid_p[x][y], 0)
                        self.update_grid(x,y,val)
                # Mark the hit point with BLACK (or other color if needed)
                if 0 <= grid_x < self.GRID_SIZE and 0 <= grid_y < self.GRID_SIZE:
 
                    #self.grid_p[grid_x][grid_y] = self.probabilistic_update(PRIOR_OCC, SENSOR_OCC, SENSOR_EMPTY, self.grid_p[grid_x][grid_y], 1)
                    val = self.probabilistic_update(PRIOR_OCC, SENSOR_OCC, SENSOR_EMPTY, self.grid_p[grid_x][grid_y], 1)
                    self.update_grid(grid_x,grid_y,val)
                ray_ids.append(ray_num)
            ray_num = ray_num+1
        self.grid = self.grid_p
        self.update_cell()

    def probabilistic_update(self, prior_occ, sensor_occ, sensor_empty, grid_val, hit):
        """
        Probabilistic update step for a singuler grid space
        """
        # Calculate log odds: Odds(x) = p(x)/(1-p(x))

        if hit == 1:
            #print(1)
            sensor_term = math.log(sensor_occ/(1-sensor_occ))
            prior_term = math.log(prior_occ/ (1-prior_occ))
        else:
            #print(0)
            sensor_term = math.log(sensor_empty/(1-sensor_empty))
            prior_term = math.log((1-prior_occ)/prior_occ)
        #print("grid VAL: " + str(grid_val))
        if grid_val == 0:
            recursive_term = 0
        else:
            #print(grid_val)
            recursive_term = math.log(grid_val)
        
        log_odds = sensor_term + recursive_term - prior_term
        # Log probability from Log Odds: p(x) = 1/(1+1/odds(x))
        #Prob from log odds: p(x) = 1-1/(1+exp(logOdds))
        #print(log_odds)
        grid_prob = 1- 1/(1+np.exp(log_odds))
        return grid_prob

    def paintEvent(self, event):
        """
        Default method for initializing a QPainter OGM
        and filling it based off of self.grid.
        """
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self.pixmap)
   
    def update_grid(self, row, col, new_value):
        if self.grid[row][col] != new_value:
            self.grid[row][col] = new_value
            self.changed_cells.add((row, col))
    
    def update_cell(self):
        painter = QPainter(self.pixmap)
        #painter = QPainter(self)
        
        # Only update the changed cells
        max_value = 1
        grayscale = 1 - self.grid  # Precompute the grayscale values
        for row, col in self.changed_cells:
            if self.prob == 0:
                painter.setBrush(COLOR_MAP[self.grid[row][col]])
            else:
                intensity = int((grayscale[row][col] / max_value) * 255)
                painter.setBrush(QColor(intensity, intensity, intensity))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRect(
                col * CELL_SIZE,
                row * CELL_SIZE,
                CELL_SIZE,
                CELL_SIZE,
        )

        # Clear changed cells after painting
        self.changed_cells.clear()
        self.update()


    def reset_map(self):
        """
        Resets the OGM to all white space.
        """
        self.grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)
        self.grid_p = np.ones((self.GRID_SIZE, self.GRID_SIZE), dtype=int) * 0.5
        self.initial = 1
        self.changed_cells.clear()
        self.update_map()

    def to_grid(self, scaling_factor, *args):
        for arg in args:
            yield int(arg // scaling_factor) + self.OFFSET
    
    def compensate_movement(self, scaling_factor, robot_pos):
        prev_robot_grid_x, prev_robot_grid_y = self.to_grid(scaling_factor, self.robot_x, self.robot_y)
        #self.grid[prev_robot_grid_x][prev_robot_grid_y] = GREY
        self.update_grid(prev_robot_grid_x, prev_robot_grid_y, WHITE)

        self.robot_x, self.robot_y, _ = robot_pos
        robot_grid_x, robot_grid_y = self.to_grid(scaling_factor, self.robot_x, self.robot_y)
        #self.grid[robot_grid_x][robot_grid_y] = RED
        self.update_grid(robot_grid_x, robot_grid_y, RED)

        return robot_grid_x, robot_grid_y
    
    def compensate_movement_1(self, scaling_factor, robot_pos):
        prev_robot_grid_x, prev_robot_grid_y = self.to_grid(scaling_factor, self.robot_x, self.robot_y)
        #self.grid[prev_robot_grid_x][prev_robot_grid_y] = GREY
        self.update_grid(prev_robot_grid_x, prev_robot_grid_y, GREY)

        self.robot_x, self.robot_y, _ = robot_pos
        robot_grid_x, robot_grid_y = self.to_grid(scaling_factor, self.robot_x, self.robot_y)
        #self.grid[robot_grid_x][robot_grid_y] = RED
        self.update_grid(robot_grid_x, robot_grid_y, RED)

        return robot_grid_x, robot_grid_y

    @staticmethod
    def bresenham(x1, y1, x2, y2):
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
