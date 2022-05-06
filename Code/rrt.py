import pygame
from rrtbase import RRTmap, Robot
from rrtbase import RRTgraph
import time
import math

class RRT:
    def __init__(self, start, goal, map_img, robot_img):
        self.dimensions = (600,1000)
        self.start = start
        self.goal = goal
        self.map_img = map_img
        self.robot_img = robot_img

    def main(self):
        iteration = 0
        t1 = 0
        pygame.init()
        map = RRTmap(self.start, self.goal, self.dimensions, self.map_img)
        graph = RRTgraph(self.start, self.goal, self.dimensions, map.map)
        # end process if goal or start collide with obstacles
        if map.map.get_at(self.start) == (0,0,0) or map.map.get_at(self.goal) == (0,0,0):
            print('Start/Goal collides with obstacle')
            return None

        map.drawMap_mod() # make start and goal circles (no obstacle)

        t1 = time.time()
        while (not graph.path_to_goal()):
            # timeout error
            elapsed = time.time() - t1
            t1 = time.time()
            # raise error if its taking too long
            if elapsed > 10:
                raise
            # bias process
            if iteration % 10 == 0:
                X,Y,Parent = graph.bias(self.goal)
                pygame.draw.circle(map.map,map.Grey,(X[-1],Y[-1]),map.nodeRad+2,0)
                pygame.draw.line(map.map,map.Blue,(X[-1], Y[-1]),
                            (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
            # expansion process
            else:
                X,Y,Parent = graph.expand()
                pygame.draw.circle(map.map,map.Grey,(X[-1],Y[-1]),map.nodeRad+2,0)
                pygame.draw.line(map.map,map.Blue,(X[-1], Y[-1]),
                            (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
            # frame update
            if iteration % 5 == 0:
                pygame.display.update()
            iteration += 1

        path_coordinates = graph.getPathCoords()
        map.drawPath(path_coordinates)
        # INTERPOLATION FOR ROBOT MOVEMENT SMOOTHNESS
        # for splitting the total path to smaller steps
        oldpath = path_coordinates
        newpath = []
        for i in range(0, len(graph.path)-1):
            if i > len(graph.path):
                break
            x1,y1 = oldpath[i]
            x2,y2 = oldpath[i+1]
            for i in range(0,5):
                u = i/5
                x = int(x2*u + x1*(1-u))
                y = int(y2*u + y1*(1-u))
                newpath.append((x,y))
        # rev_path_coordinates = path_coordinates[::-1]
        rev_path_coordinates = newpath[::-1]

        # Robot Movement Simulation
        robot = Robot(self.start, 23, self.robot_img) # robot object
        # save map image
        pygame.image.save(map.map, "path_planned.png")
        temp_map = map.map.copy() # temporary copy of the map (without robot)
        time_step = 0.08 # seconds

        total_dist = 0 # total distance traversed by the robot
        total_time = 0 # total time taken to cover the distance
        for i in range(0,len(rev_path_coordinates)-1):
            pygame.display.update()
            map.map.blit(temp_map, (0,0)) # draw the temp map
            # calculating Angle of Vehicle
            dx = abs(rev_path_coordinates[i+1][0] - rev_path_coordinates[i][0])
            dy = abs(rev_path_coordinates[i+1][1] - rev_path_coordinates[i][1])
            # overall distance between between the two points 
            point_dist = math.sqrt(dx**2 + dy**2)
            vel = point_dist/time_step # pixels/second
            # adding the distance between the points to total distance
            total_dist += point_dist
            # adding time step in each step to total time taken
            total_time += time_step
            psi = math.atan2(dy,dx) # arc tangent (in radians)
            if rev_path_coordinates[i+1][0] > rev_path_coordinates[i][0]: #x2 > x1
                if rev_path_coordinates[i+1][1] > rev_path_coordinates[i][1]:
                    theta = -psi
                else:
                    theta = psi
            else:
                if rev_path_coordinates[i+1][1] > rev_path_coordinates[i][1]:
                    theta = math.pi + psi
                else:
                    theta = math.pi - psi
            # print velocity and angle on console
            # print("Velocity", int(vel), "Angle", round(math.degrees(theta), 2))

            # print text data on screen
            txt = f"Vel: {int(vel)} px/s Angle: {round(math.degrees(theta), 1)}"
            text = map.font.render(txt,True,map.Black,(127,255,0))
            textRect = text.get_rect(center = (map.Mapw-180, map.Maph-550))
            map.map.blit(text, textRect)   # draw text     
            # draw Robot on screen
            # rotate robot
            rotated = pygame.transform.rotozoom(robot.robot_img, math.degrees(theta), 1) 
            # get rectangle object at new coordinates for robot
            rect = robot.robot_rect(rev_path_coordinates[i][0],rev_path_coordinates[i][1]) 
            map.map.blit(rotated, rect) # draw robot
            # pygame.display.update()
            time.sleep(time_step)

        # robot final position print (as last coordinate is ignored in the above loop)
        map.map.blit(temp_map, (0,0)) # reset the map 
        rotated = pygame.transform.rotozoom(robot.robot_img, 0, 1) # rotate robot
        # get rect object at new coordinates
        rect = robot.robot_rect(rev_path_coordinates[-1][0],rev_path_coordinates[-1][1])
        map.map.blit(rotated, rect) # draw robot
        pygame.display.update()
        # Coordinates Output as text file
        coords_str_list = []
        for elem in path_coordinates[::-1]:
            coords_str_list.append(str(elem))
        f = open("path_coords.txt", "w")
        f.writelines(coords_str_list)
        f.close()
        pygame.display.update()

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            
        # ------------------    END OF MAIN 
        return path_coordinates[::-1], round(total_dist,2), round(total_time,1)

#------------------------------------------------------------
# RUN EVERYTHING FROM HERE

# rrt = RRT((50,50),(930,550), "ObstacleMap2.png", "DDR.png")
# if __name__ == '__main__':
#     # main()
#     result = False
#     while not result:
#         try:
#             coords_of_path = rrt.main()
#             result = True
#         except:
#             result = False