import random
import math
import pygame

class Robot:
    def __init__(self, start_pos, width, robot_img_path):
        self.width = width
        self.x = start_pos[0]
        self.y = start_pos[1]
        self.m2p = 3779.52 #unit conversion metre to pixel
        self.angle = 0 # angle initialization

        self.robot_img = pygame.image.load(robot_img_path)
    
    def robot_rect(self, x, y):
        '''
        Gives a rectangle object for the robot at x and y coordinates
        '''
        return self.robot_img.get_rect(center = (x,y))



class RRTmap:
    def __init__(self, start, goal, MapDimensions, map_img_path):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        # window settings
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))

        # self.map.fill((255,255,255)) # white background

        # text variables
        self.font = pygame.font.Font('freesansbold.ttf', 25)
        # self.text = self.font.render('default',True,self.White, self.Black)
        # self.textRect = self.text.get_rect()
        # self.textRect.center = (self.Maph-100,self.Mapw-100)        

        self.map_img = pygame.image.load(map_img_path) # load bg image
        # self.robot_img = pygame.image.load(robot_img_path) # load robot image
        self.map.blit(self.map_img, (0,0)) # draw bg image
        # self.map.blit(self.robot_img, (self.start[0]-13,self.start[1]-12)) # draw robot (temporarily)

        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        # obstacles
        self.obstacles = []
        # self.obsdim = obsdim
        # self.obsNumber = obsnum

        # colors
        self.Grey = (70,70,70)
        self.Blue = (0,0,255)
        self.Green = (0,255,0)
        self.Red = (255,0,0)
        self.White = (255,255,255)
        self.Black = (0,0,0)
        self.Yellow = (255,255,0)

    def drawMap(self,obstacles):
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad+5,0)
        pygame.draw.circle(self.map, self.Red, self.goal, self.nodeRad+20,1)
        self.drawObs(obstacles)

    def drawMap_mod(self):
        '''
        Draw start and goal circles
        '''
        # draw start circle
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad+5,0)
        # draw goal circle
        pygame.draw.circle(self.map, self.Red, self.goal, self.nodeRad+20,1)

    def drawPath(self, path):
        '''
        Draw the nodes within the path
        '''
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, self.nodeRad+3, 0)

    def drawObs(self, obstacles):
        '''
        Draw obstacles from a list of rectangle objects
        '''
        obstaclesList = obstacles.copy()
        while (len(obstaclesList)) >0:
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.Grey, obstacle)

class RRTgraph:
    def __init__(self, start, goal, MapDimensions, map):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        self.map = map # surface obj
        # intialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        # the obstacles
        self.obstacles = []
        # self.obsDim = obsdim
        # self.obsNum = obsnum
        # path
        self.goalstate = None
        self.path = []

    def makeRandomRect(self):
        '''
        Randomly generate upper corner coordinates of a rectangular obstacle.
        '''
        uppercornerx = int(random.uniform(0, self.mapw-self.obsDim))
        uppercornery = int(random.uniform(0, self.maph-self.obsDim))

        return (uppercornerx, uppercornery)

    def makeobs(self):
        '''
        Make obstacles using the randomly generated upper corner coordinates from
        makeRandomRect function.
        '''
        obs = []
        for i in range(0, self.obsNum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                # run until a rectangle that does not collide with start/goal point
                # is created
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsDim,self.obsDim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs

    def add_node(self,n,x,y):
        '''
        Add a node with coordinates x and y at index n
        '''
        self.x.insert(n,x)
        self.y.insert(n,y)

    def remove_node(self,n):
        '''
        Remove the node at index n
        '''
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        '''
        Add edge (relationship or link) between the child and parent nodes
        '''
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        '''
        Remove link between the node at n and parent
        '''
        self.parent.pop(n)

    def number_of_nodes(self):
        '''
        Number of nodes in the tree
        '''
        return len(self.x)
        
    def distance(self, n1, n2):
        '''
        Calculate the Euclidean distance between two nodes
        '''
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        return math.sqrt((float(x1)-float(x2))**2 + (float(y1)-float(y2))**2)

    def sample_envir(self):
        '''
        Randomly generate a sample within the map and return its coordinates
        '''
        x = int(random.uniform(0,self.mapw))
        y = int(random.uniform(0,self.maph))
        return x, y

    def nearest(self, n):
        '''
        Find the nearest node to the given node and return its index
        '''
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0,n):
            if self.distance(i,n) < dmin:
                dmin = self.distance(i,n)
                nnear = i
        return nnear

    def isFree(self):
        '''
        Test if a node is in free space or overlapping with an obstacle
        '''
        n = self.number_of_nodes()-1 # id/index of the last node
        (x,y) = (self.x[n],self.y[n])
        obs = self.obstacles.copy()
        while len(obs)>0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True

    def isFree_mod(self):
        '''
        Test if a node is in free space or overlapping with an obstacle (modification)
        '''
        n = self.number_of_nodes()-1 # id/index of the last node
        (x,y) = (self.x[n],self.y[n])
        # obs = self.obstacles.copy() # list of obstacles

        #------------------------------------
        # check if x,y is on a black pixel (obstacle)
        if self.map.get_at((x,y)) == (0,0,0):
            self.remove_node(n)
            return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        '''
        Check if an edge (link) between two nodes cross any obstacle
        '''
        obs = self.obstacles.copy()
        while len(obs)>0:
            rectang = obs.pop(0)
            for i in range(0,101):
                # sampling across the line 100 times
                u = i/100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if rectang.collidepoint(x,y):
                    return True
        return False

    def crossObstacle_mod(self, x1, x2, y1, y2):
        '''
        Check if an edge (link) between two nodes cross any obstacle (modification)
        '''
        for i in range(0,101):
            # sampling across the line 100 times
            u = i/100
            x = int(x1*u + x2*(1-u))
            y = int(y1*u + y2*(1-u))

            # print('err check:',self.map.get_at((x,y)))
            # print('err check2:',(x,y), type(x))
            #------------------------------------------
            # check if the color at surface at (x,y) is black
            if self.map.get_at((x,y)) == (0,0,0):
                return True
        return False

    def connect(self, n1, n2):
        '''
        Connect two nodes if the edge is not crossing an obstacle and return True.
        If the edge is crossing an obstacle, return False.
        '''
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        if self.crossObstacle_mod(x1,x2,y1,y2):
            return False
        else:
            self.add_edge(n1,n2)
            return True

    def step(self, nnear, nrand, dmax = 35):
        '''
        Create a new node at a distance of dmax between the newly created (nrand) temporary node
        and its nearest node. nrand is deleted. The new node takes its place. If goal is near
        then goal node takes the place of new node.
        nnear = Nearest node index
        nrand = Newly created node index
        '''
        d = self.distance(nnear, nrand)
        if d> dmax:
            u = dmax/d
            (xnear,ynear) = (self.x[nnear],self.y[nnear])
            (xrand,yrand) = (self.x[nrand],self.y[nrand])
            (px,py) = (xrand - xnear,yrand - ynear)
            theta = math.atan2(py,px)
            # new coordinates
            (x,y) = (int(xnear+dmax*math.cos(theta)),int(ynear+dmax*math.sin(theta)))
            # the randomly created node is removed
            self.remove_node(nrand)
            # check if the new node is within range (radius = dmax)
            # of the goal
            if (abs(x-self.goal[0]) < 35) and (abs(y-self.goal[1]) < 35):
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)
            
    def bias(self, ngoal):
        '''
        Bias: expansion in the direction towards the goal
        '''
        n = self.number_of_nodes()
        self.add_node(n,ngoal[0],ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        connection_check = self.connect(nnear, n)
        if not connection_check:
            self.remove_node(n)
        return self.x, self.y, self.parent


    def expand(self):
        '''
        Random expansion of the tree using random samples
        '''
        n = self.number_of_nodes()
        x,y = self.sample_envir()
        self.add_node(n,x,y)
        if self.isFree_mod():
            xnearest = self.nearest(n)
            self.step(xnearest,n)
            connection_check = self.connect(xnearest,n)
            if not connection_check:
                self.remove_node(n)
        return self.x, self.y, self.parent

    def path_to_goal(self):
        '''
        Add the path from goal to start as a list (nodes in reverse from goal to start)
        if goal was reached. Returns boolean variable (goalFlag) as True if goal was reached
        and as False if not reached.
        '''
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        '''
        Get path coordinates
        '''
        pathCoords = []
        for node in self.path:
            x,y = (self.x[node], self.y[node])
            pathCoords.append((x,y))
        return pathCoords
        

    def cost(self):
        pass