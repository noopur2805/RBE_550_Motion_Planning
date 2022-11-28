import global_v
import math
import world
import random

class Robot:
    def __init__(self, width, length, wheelbase, turning_radius, max_velocity, position, heading): ## Heading is in radians
        self.color = global_v.BLUE
        self.width = width
        self.length = length
        self.wheelbase = wheelbase
        self.turning_radius = turning_radius
        self.max_velocity = max_velocity
        self.position = position
        # self.x = self.position[0]
        # self.y = self.position[1]
        self.heading = heading
        self.robot_points = [[3, 1], [3, -1], [-1, -1], [-1, 1]]
        self.robot_lines = []
        self.rasterized_lines = []
        self.robot = []

    def make_end_point_pairs(self):
        self.robot_lines.clear()
        for j in range(len(self.robot_points)):
            line = []
            line.append(self.robot_points[j])
            if (j == len(self.robot_points) - 1):
                line.append(self.robot_points[0])
            else:
                line.append(self.robot_points[j + 1])
            self.robot_lines.append(line)

    def make_bresenham_lines(self):
        self.rasterized_lines.clear()
        for i in self.robot_lines:
            x, y = i[0]
            x2, y2 = i[1]
            line = []
            line.append(i[0])
            dx = abs(x2 - x)
            dy = -abs(y2 - y)
            sx = 1 if x < x2 else -1
            sy = 1 if y < y2 else -1
            err = dx + dy
            while (x != x2 or y != y2):
                e2 = 2*err
                if (e2 >= dy):
                    err += dy
                    x += sx
                if (e2 <= dx):
                    err += dx
                    y+=sy
                temp = x, y
                line.append(temp)
            line.append(i[1])
            self.rasterized_lines.append(line)

    def make_filled_polygon(self):
        block_size = global_v.SIZE // global_v.ROWS
        self.robot.clear()
        x0 = []
        x1 = []
        for t in range(global_v.ROWS):
            x0.append(-1)
            x1.append(-1)
        for rasterized_line in self.rasterized_lines:
            for element in rasterized_line:
                x, y = element
                if (x0[y] == -1):
                    x0[y] = x
                    x1[y] = x
                else:
                    x0[y] = min(x0[y], x)
                    x1[y] = max(x1[y], x)
        n=0
        for y in range(global_v.ROWS):
            if (x0[y]!= -1):
                self.robot.append([])
                for x in range(x0[y], x1[y]):
                        block = world.Block(x, y, self.color, block_size, None)
                        self.robot[n].append(block)
                block = world.Block(x1[y], y, self.color, block_size, None)
                self.robot[n].append(block)
                n = n+1
                
        
        x0.clear()
        x1.clear()

    def make_robot(self):
        points = [[3, 1], [3, -1], [-1, -1], [-1, 1]]
        for i, point in enumerate(points):
            a = point[0] * math.cos(self.heading) - point[1] * math.sin(self.heading)
            b = point[0] * math.sin(self.heading) + point[1] * math.cos(self.heading)
            self.robot_points[i][0] = round(self.position[0] + a)
            self.robot_points[i][1] = round(self.position[1] + b)
            if self.robot_points[i][0] < 0 or self.robot_points[i][0] >= global_v.ROWS or self.robot_points[i][1] < 0 or self.robot_points[i][1] >= global_v.ROWS:
                return True

        self.make_end_point_pairs()
        self.make_bresenham_lines()
        self.make_filled_polygon()
        return False
        
    def draw_robot(self, grid):
        for i in range(len(self.robot)):
            for j in range(len(self.robot[i])):
                grid[self.robot[i][j].row][self.robot[i][j].col].color = self.robot[i][j].color
        grid[self.position[0]][self.position[1]].color = global_v.ROBOT_CENTER
    
    def clear_robot(self, grid):
        for i in range(len(self.robot)):
            for j in range(len(self.robot[i])):
                grid[self.robot[i][j].row][self.robot[i][j].col].color = global_v.WHITE

    def check_collision(self, grid):
        for i in range(len(self.robot)):
            for j in range(len(self.robot[i])):
                block_color = grid[self.robot[i][j].row][self.robot[i][j].col].color
                if (block_color == global_v.GREEN or block_color == global_v.RED or block_color == global_v.BLACK):
                    # print(self.robot[i][j].row, self.robot[i][j].col, grid[self.robot[i][j].row][self.robot[i][j].col].color, "Point of collision")
                    return True
        return False

## Returns true if the configuration is possible
    def check_possible_config(self, grid, position, heading):
        temp_robot = Robot(self.width, self.length, self.wheelbase, self.turning_radius, self.max_velocity, position, heading)
        value = temp_robot.make_robot()
        if (temp_robot.check_collision(grid) == False and value == False):
            return True
        else:
            return False



def spawn_robot(grid):
    condition = True
    while condition:    
        x_position = random.randint(10, global_v.ROWS-10)
        y_position = random.randint(10, global_v.ROWS-10)
        position = x_position, y_position
        heading = random.randint(0, 360)
        heading = heading * math.pi/180
        robot = Robot(global_v.width, global_v.length, global_v.wheelbase, global_v.turning_radius, global_v.max_velocity, position, heading)
        robot.make_robot()
        if (robot.check_collision(grid) == False):
            robot.draw_robot(grid)
            # self.draw()
            condition = False
            return robot