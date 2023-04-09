import math
import random
from time import perf_counter
import pygame



class WindowDraw(object):
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph,self.Mapw = self.MapDimensions

        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw,self.Maph))
        self.map.fill((255,255,255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        self.grey = (70,70,70)
        self.Blue = (0,0,255)
        self.Green = (0,255,0)
        self.Red = (255,0,0)
        self.white = (255,255,255)

    def drawMap(self,obstacles):
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad+5,0)
        pygame.draw.circle(self.map, self.Red, self.goal, self.nodeRad+15,1)
        self.drawObs(obstacles)


    def drawPath(self,path):
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, self.nodeRad+3,0)

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey,obstacle)

class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        #self.pytohn_sum = 0.0;
        (x,y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.MapDimensions = MapDimensions
        self.Maph,self.Mapw = self.MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        self.goalState = None
        self.path = []

    def add_node(self,n,x,y):
        self.x.insert(n,x)
        self.y.append(y)

    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self,parent,child):
        self.parent.insert(child,parent)

    def remove_edge(self,n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self,n1,n2):
        (x1,y1) = (self.x[n1], self.y[n1])
        (x2,y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px+py) ** (0.5)

    def sample_envir(self):
        x = int(random.uniform(0,self.Mapw))
        y = int(random.uniform(0,self.Maph))
        return x,y

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x,y) = (self.x[n],self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True

    def connect(self,n1,n2):
        (x1,y1) = (self.x[n1], self.y[n1])
        (x2,y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True

    def nearest(self, n):
        dmin = self.distance(0,n)
        nnear = 0;
        for i in range(0,n):
            if self.distance(i,n) < dmin:
                dmin = self.distance(i,n)
                nnear = i
        return nnear
    
    def step(self, nnear, nrand, dmax = 35):
        start = perf_counter()
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax/d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px,py) = (xrand-xnear,yrand-ynear)
            theta = math.atan2(py,px)
            (x,y) = (int(xnear+dmax * math.cos(theta)),
                     int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            if abs(x-self.goal[0]) < dmax and abs(y-self.goal[1]) < dmax :
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalState = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)
        print('{} took {:.7f} seconds\n\n'.format("step: ", perf_counter() - start))

    #def getPythonSum(self):
        #return self.pytohn_sum

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalState)
            newpos = self.parent[self.goalState]
            while (newpos != 0):
                self.path.append(newpos)
            self.path.append(0)
        return self.goalFlag

    def getPathCoors(self):
        pathCoords=[]
        for node in self.path:
            x,y = (self.x[node], self.y[node])
            pathCoords.append((x,y))
        return pathCoords

    def bias(self,ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear,n)
        self.connect(nnear,n)
        #self.time_test(lambda d: [self.step()], '[RRTAlg (Python implementation)]')
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x,y = self.sample_envir()
        self.add_node(n,x,y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest,n)
            self.connect(xnearest,n)
        return self.x, self.y, self.parent


    def crossObstacle(self,x1,x2,y1,y2):
        obs = self.obstacles.copy()
        while(len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0,101):
                u = i/100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if rectang.collidepoint(x,y):
                    return True
        return False



    def makeRandomRRect(self):
        uppercornerx = int(random.uniform(0,self.Mapw-self.obsdim))
        uppercornery = int(random.uniform(0,self.Maph-self.obsdim))
        return (uppercornerx,uppercornery)
    
    def makeobs(self):
        obs = []

        for i in range(0,self.obsnum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRRect()
                rectang = pygame.Rect(upper,(self.obsdim, self.obsdim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs
