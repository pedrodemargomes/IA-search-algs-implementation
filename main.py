import sys
from math import dist
import matplotlib.pyplot as plt

class Polygon:
    def __init__(self, number, vertexes):
        self.number = number
        self.vertexes = vertexes


def readPoints():
    pointsDict = {}
    for line in sys.stdin:
        spltline = line.split()
        if line == "\n":
            return pointsDict
        pointsDict[spltline[0]] = (float(spltline[1]), float(spltline[2]))

def readPolygons(pointsDict):
    polygons = []
    for line in sys.stdin:
        if line == "\n":
            return polygons
        spltline = line.split()
        number = int(spltline[0])
        vertexes = []
        for v in spltline[1:]:
            vertexes.append(pointsDict[v])
        polygons.append(Polygon(number, vertexes))

def ccw(A,B,C):
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def intersectPolygons(A, B, polygons):
    for polygon in polygons:
        for i in range(0, len(polygon.vertexes)-1):
            if intersect(A, B, polygon.vertexes[i], polygon.vertexes[i+1]):
                return True
    return False

def neighbours(point, polygons, destPoint):
    neighbours = []
    for polygon in polygons:
        if point in polygon.vertexes:
            idx = polygon.vertexes.index(point)
            if idx == 0:
                neighbours.append(polygon.vertexes[len(polygon.vertexes)-2])
                neighbours.append(polygon.vertexes[1])
            elif idx == len(polygon.vertexes)-1:
                neighbours.append(polygon.vertexes[idx-1])
                neighbours.append(polygon.vertexes[1])
            else:
                neighbours.append(polygon.vertexes[idx-1])
                neighbours.append(polygon.vertexes[idx+1])
            continue
        for i in range(0, len(polygon.vertexes)-1):
            # print("neig point test: ", polygon.vertexes[i])
            if not intersectPolygons(point, polygon.vertexes[i], polygons):
                neighbours.append(polygon.vertexes[i])
    
    if not intersectPolygons(point, destPoint, polygons):
        neighbours.append(destPoint)

    return neighbours

def BFS(initPoint, destPoint, polygons):
    queue = []
    visitedPoints = []
    parent = {}
    visitedPoints.append(initPoint)
    queue.append((initPoint, 0))

    while queue:
        queue = sorted(queue, key=lambda tup: tup[1])
        p = queue.pop(0)
        point = p[0]
        g = p[1]
        # print("Ponto atual: ", point)
        if point == destPoint:
            break

        # print("Vizinhos: ", end = "")
        for n in neighbours(point, polygons, destPoint):
            if n not in visitedPoints:
                visitedPoints.append(n)
                parent[n] = point
                # print(n, end=", ")
                queue.append((n, g+dist(point,n)))
        # print("")
    print("Vertices explorados: ", len(visitedPoints))
    path = []
    d = destPoint
    cost = 0
    while d != initPoint:
        path.insert(0, d)
        cost += dist(d, parent[d])
        d = parent[d]
    path.insert(0, initPoint)
    print("Caminho: ", path)
    print("Custo: ", cost)

numberVisitedPointsID = 0
def id_recur(curPoint, destPoint, polygons, max_depth, path):
    global numberVisitedPointsID
    numberVisitedPointsID += 1
    if curPoint == destPoint:
        return True

    if max_depth <= 0:
        return False

    for n in neighbours(curPoint, polygons, destPoint):
        if n not in path:
            path.append(n)
            if(id_recur(n, destPoint, polygons, max_depth-1, path)):
                return True
            path.pop()
    
    return False

def id(initPoint, destPoint, polygons, max_depth):
    global numberVisitedPointsID
    numberVisitedPointsID = 0
    for i in range(1, max_depth):
        path = [initPoint]
        if id_recur(initPoint, destPoint, polygons, i, path):
            print("Numero vertices explorados: ", numberVisitedPointsID)
            print("Caminho: ", path)
            cost = 0
            for j in range(0, len(path)-1):
                cost += dist(path[j], path[j+1])
            print("Custo: ", cost)
            print("Numero de iteracoes: ", i)
            return True
    print("Nenhum caminho encontrado")
    return False

def heuristic(point, destPoint):
    return dist(point, destPoint)

def astar(initPoint, destPoint, polygons):
    queue = []
    visitedPoints = []
    parent = {}
    visitedPoints.append(initPoint)
    queue.append((initPoint, 0))

    while queue:
        queue = sorted(queue, key=lambda tup: tup[1])
        p = queue.pop(0)
        point = p[0]
        g = p[1]
        # print("Ponto atual: ", point)
        if point == destPoint:
            break

        # print("Vizinhos: ", end = "")
        for n in neighbours(point, polygons, destPoint):
            if n not in visitedPoints:
                visitedPoints.append(n)
                parent[n] = point
                # print(n, end=", ")
                queue.append((n, g+dist(point,n)+heuristic(n, destPoint)))
        # print("")
    print("Vertices explorados: ", len(visitedPoints))
    path = []
    d = destPoint
    cost = 0
    while d != initPoint:
        path.insert(0, d)
        cost += dist(d, parent[d])
        d = parent[d]
    path.insert(0, initPoint)
    print("Caminho: ", path)
    print("Custo: ", cost)

numberVisitedPointsIDASTAR = 0
def idastar(initPoint, destPoint, polygons):
    threshold = heuristic(initPoint, destPoint)
    global numberVisitedPointsIDASTAR
    numberVisitedPointsIDASTAR = 0
    numIter = 1
    while True:
        path = [initPoint]
        distance = idastar_rec(initPoint, destPoint, polygons, threshold, 0, path)
        if distance == float("inf"):
            return -1
        elif distance < 0:
            print("Vertices explorados: ", numberVisitedPointsIDASTAR)
            print("Caminho: ", path)
            print("Custo: ", -distance)
            print("Numero de iteracoes: ", numIter)
            return -distance
        else:
            threshold = distance
        numIter += 1

def idastar_rec(curPoint, destPoint, polygons, threshold, distance, path):
    global numberVisitedPointsIDASTAR
    numberVisitedPointsIDASTAR += 1
    if curPoint == destPoint:
        return -distance

    estimate = distance + heuristic(curPoint, destPoint)
    if estimate > threshold:
        return estimate

    min = float("inf")
    for n in neighbours(curPoint, polygons, destPoint):
        if n not in path:
            path.append(n)
            val = idastar_rec(n, destPoint, polygons, threshold, distance+dist(curPoint, n), path)
            if val < 0:
                return val
            elif val < min:
                min = val
            path.pop()

    return min

pointsDict = readPoints()
polygons = readPolygons(pointsDict)

tmp = sys.stdin.readlines()

initPoint = (float(tmp[0].split()[0]), float(tmp[0].split()[1]))
destPoint = (float(tmp[1].split()[0]), float(tmp[1].split()[1]))

# Buscas

print("BFS: ")
BFS(initPoint, destPoint, polygons)
print("\nID: ")
id(initPoint, destPoint, polygons, 20)
print("\nA*: ")
astar(initPoint, destPoint, polygons)
print("\nIDA*: ")
idastar(initPoint, destPoint, polygons)

# Plot polygons
for p in polygons:
    coord = [list(x) for x in p.vertexes]
    xs, ys = zip(*coord) #create lists of x and y values
    plt.plot(xs,ys)
    plt.grid()
plt.show() # if you need...







