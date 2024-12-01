import math
class PurePursuit:
    def __init__(self,jackal_robot,path,lookahead=0.5):
        stuff=True
        self.path=path
        self.lookahead=lookahead
        self.jackal_robot=jackal_robot
    def find_lookahead_point(self):
        robot_position=self.jackal_robot.localize()
        for i in range(len(self.path) - 1):
            start = self.path[i]
            end = self.path[i + 1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            fx = start[0] - robot_position[0]
            fy = start[1] - robot_position[1]
            a = dx ** 2 + dy ** 2
            b = 2 * (fx * dx + fy * dy)
            c = fx ** 2 + fy ** 2 - self.lookahead ** 2
            discriminant = b ** 2 - 4 * a * c

            if discriminant >= 0:
                discriminant_sqrt = math.sqrt(discriminant)
                t1 = (-b + discriminant_sqrt) / (2 * a)
                t2 = (-b - discriminant_sqrt) / (2 * a)

                if 0 <= t1 <= 1:
                    return (start[0] + t1 * dx, start[1] + t1 * dy)
                if 0 <= t2 <= 1:
                    return (start[0] + t2 * dx, start[1] + t2 * dy)

        return self.path[-1]

        

    
    
    
    