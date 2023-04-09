from itertools import islice
from random import random
from time import perf_counter
import time
from WindowDrawBase import WindowDraw
from WindowDrawBase import RRTGraph
import pygame

COUNT = 500000  # Change this value depending on the speed of your computer
DATA = list(islice(iter(lambda: (random() - 0.5) * 3.0, None), COUNT))

e = 2.7182818284590452353602874713527


def sinh(x):
    return (1 - (e ** (-2 * x))) / (2 * (e ** -x))


def cosh(x):
    return (1 + (e ** (-2 * x))) / (2 * (e ** -x))


def tanh(x):
    tanh_x = sinh(x) / cosh(x)
    return tanh_x


def test(fn, name):

    start = perf_counter()
    result = fn(DATA)
    duration = perf_counter() - start
    print('{} took {:.3f} seconds\n\n'.format(name, duration))

    for d in result:
        assert -1 <= d <= 1, " incorrect values"


def main():
    print('Running benchmarks with COUNT = {}'.format(COUNT))

    test(lambda d: [tanh(x) for x in d], '[tanh(x) for x in d] (Python implementation)')
    
    from superfastcode import fast_tanh
    test(lambda d: [fast_tanh(x) for x in d], '[fast_tanh(x) for x in d] (CPython C++ extension)')
    
    dimensions = (600, 1000)
    start = (50,50)
    goal = (510,510)
    obsdim = 30
    obsnum = 50
    iteration = 0
    t1 = 0

    pygame.init()
    map = WindowDraw(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = graph.makeobs()
    map.drawMap(obstacles)

    #while(True):
    #    x,y = graph.sample_envir()
    #    n = graph.number_of_nodes()
    #    graph.add_node(n,x,y)
    #    graph.add_edge(n-1,n)
    #    x1,y1 = graph.x[n], graph.y[n]
    #    x2,y2 = graph.x[n-1], graph.y[n-1]
    #    if(graph.isFree()):
    #        pygame.draw.circle(map.map, map.Red,(graph.x[n], graph.y[n]), map.nodeRad, map.nodeThickness)
    #        if not graph.crossObstacle(x1,x2,y1,y2):
    #            pygame.draw.line(map.map,map.Blue, (x1,y1),(x2,y2), map.edgeThickness)
            
    #    pygame.display.update()

    t1=time.time()
    while(not graph.path_to_goal()):
        elapsed = time.time()-t1
        t1=time.time()
        if elapsed > 10:
            raise

        if iteration % 10 == 0:
            X,Y,Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2,0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)
        else:
             X,Y,Parent = graph.expand()
             pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad+2,0)
             pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)
        if iteration % 5 == 0:
           pygame.display.update()
        iteration += 1
    #print("Python steps took: ", graph.getPythonSum())
    map.drawPath(graph.getPathCoors())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)
    



if __name__ == '__main__':
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False


    #from superfastcode2 import fast_tanh2
    #test(lambda d: [fast_tanh2(x) for x in d], '[fast_tanh2(x) for x in d] (PyBind11 C++ extension)')
