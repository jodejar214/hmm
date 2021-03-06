import heapq
import math
import rospy
import numpy as np

N = math.radians(90)
S = math.radians(270)
E = math.radians(0)
W = math.radians(180)
NE = math.radians(45)
SE = math.radians(315)
NW = math.radians(135)
SW = math.radians(225)

class Cell(object):
    """
    Initializes new cell with specified x,y coordinates,
    previous cell in path, and functions for cell cost
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.previous = None
        self.g = 0
        self.h = 0
        self.f = 0

class AStar(object):

    """
    Initializes AStar object with empty grid and necessary
    variables for path search using human aware motion
    planning costs for visibility and safety
    """
    def __init__(self):
        #path search
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = set()
        self.start = None
        self.end = None

        #grid init
        self.cells = {}
        self.min_height = None
        self.min_width = None
        self.max_height = None
        self.max_width = None
        self.diff = 0.0
        self.range_width = None
        self.range_height = None
        self.sum_cost = None

        #tracking and avoidance
        self.changed = []
        self.trans = {}
        self.emission = {}
        self.prevLoc = None
        self.initProb = 0.0

    """
    Initializes grid robot will travel in and assigns cost
    to each position in grid. Also gets the start position
    and determines the best end position without taking into
    account the human obstacle.
    """
    def init_grid(self, min_width, min_height, max_width, max_height, diff, start, goal, theta, human):
        #dimensions and intervals of grid
        self.min_height = min_height
        self.min_width = min_width
        self.max_height = max_height
        self.max_width = max_width
        self.diff = diff
        self.range_width = np.arange(min_width, max_width, diff)
        self.range_height = np.arange(min_height, max_height, diff)

        #get costs for cells in grid
        self.sum_cost = {}
        for x in self.range_width:
            for y in self.range_height:
                self.cells[(x,y)] = Cell(x, y)
                self.sum_cost[(x,y)] = self.getCost(x, y, goal, theta)
        self.start = self.cells[start]

        #find human
        self.sum_cost[human] += 100.0

        #find min cost point for destination
        min_coor = start
        min_cost = self.sum_cost[min_coor]
        for x in self.range_width:
            for y in self.range_height:
                if self.sum_cost[(x,y)] < min_cost:
                    min_coor = (x,y)
                    min_cost = self.sum_cost[min_coor]
        self.end = self.cells[min_coor]

        #get probs for HMM
        self.initProb = 1.0
        self.trans, self.emission = self.makeProbs()

    """
    Calculates the cost of each cell in the grid using 
    sum of distance and visibility cost functions
    """
    def getCost(self, x, y, goal, theta):
        #get distance cost of point
        xtarget = goal[0]
        ytarget = goal[1]

        dist = math.sqrt(((x - xtarget)**2) + ((y - ytarget)**2))
        dist_cost = 0
        if dist >= 1.0:
            dist_cost = dist
        elif dist == 0.0:
            dist_cost = 100.0
        else:
            dist_cost = 50.0 / dist

        #get visibility cost of point
        angle = abs(math.atan2(y-ytarget, x-xtarget) - theta)
        vis_cost = angle

        sum_cost = dist_cost + vis_cost      
        return sum_cost

    """
    Returns a list the neighboring cells of a cell which
    include the diagonals, left, right, up, and down 
    neighbors.
    """
    def get_neighbors(self, cell):
        neighbors = []
        #not on right border
        if cell.x < self.max_width-self.diff:
            neighbors.append(self.cells[(cell.x+self.diff, cell.y)])
        #not on right border and bottom border
        if cell.x < self.max_width-self.diff and cell.y > self.min_height:
            neighbors.append(self.cells[(cell.x+self.diff, cell.y-self.diff)])
        #not on bottom border
        if cell.y > self.min_height:
            neighbors.append(self.cells[(cell.x, cell.y-self.diff)])
        #not on left border and bottom border
        if cell.x > self.min_width and cell.y > self.min_height:
            neighbors.append(self.cells[(cell.x-self.diff, cell.y-self.diff)])
        #not on left border
        if cell.x > self.min_width:
            neighbors.append(self.cells[(cell.x-self.diff, cell.y)])
        #not on left border and top border
        if cell.x > self.min_width and cell.y < self.max_height-self.diff:
            neighbors.append(self.cells[(cell.x-self.diff, cell.y+self.diff)])
        #not on top border
        if cell.y < self.max_height-self.diff:
            neighbors.append(self.cells[(cell.x, cell.y+self.diff)])
        #not on right border and top border
        if cell.x < self.max_width-self.diff and cell.y < self.max_height-self.diff:
            neighbors.append(self.cells[(cell.x+self.diff, cell.y+self.diff)])
        return neighbors

    """
    Returns the path from start position to end position
    """
    def get_path(self):
        cell = self.end
        path = [(cell.x, cell.y)]
        while cell.previous is not self.start:
            cell = cell.previous
            path.append((cell.x, cell.y))

        # path.append((self.start.x, self.start.y))
        path.reverse()

        #reset for next search
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = set()
        for x in self.range_width:
            for y in self.range_height:
                self.cells[(x,y)].previous = None
                self.cells[(x,y)].g = 0
                self.cells[(x,y)].h = 0
                self.cells[(x,y)].f = 0
        return path

    """
    Update adjacent cell.
    """
    def update_cell(self, adj, cell):
        adj.g = cell.g + self.sum_cost[(adj.x,adj.y)]
        adj.h = math.sqrt(((adj.x - self.end.x)**2) + ((adj.y - self.end.y)**2))
        adj.previous = cell
        adj.f = adj.h + adj.g

    """
    Searches for the minimum cost path to the destination
    """
    def search(self):
        heapq.heappush(self.opened, (self.start.f, self.start))

        while len(self.opened):
            f, cell = heapq.heappop(self.opened)
            self.closed.add(cell)

            if cell is self.end:
                self.open = []
                self.close = set()
                return self.get_path()

            #look at neighbors of cell
            adj_cells = self.get_neighbors(cell)
            for adj_cell in adj_cells:
                if adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found
                        # for this adj cell.
                        adj = (adj_cell.x,adj_cell.y)
                        if adj_cell.g > cell.g + self.sum_cost[adj]:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))

    """
    Make transition and emission probabilities for HMM
    """
    def makeProbs(self):
        trans = {}
        emission = {}

        dirs = [N,NE,E,SE,S,SW,W,NW]
        for x in self.range_width:
            for y in self.range_height:
                neighbors = self.get_neighbors(self.cells[(x,y)])
                for n in neighbors:
                    trans[((n.x,n.y),(x,y))] = 1.0 / (len(neighbors) +  1)
                trans[((x,y),(x,y))] = 1.0 / (len(neighbors) +  1)

                for d in dirs:
                    neighborsDir, neighborF = self.neighbors_dir(self.cells[(x,y)], d)
                    for n in neighbors:
                        if neighborF != None:
                            if n == neighborF:
                                emission[(((n.x,n.y),(x,y)),d)] = 0.6
                            if n in neighborsDir:
                                emission[(((n.x,n.y),(x,y)),d)] = 0.2/len(neighborsDir)
                            else:     
                                emission[(((n.x,n.y),(x,y)),d)] = 0.2/(len(neighbors) - len(neighborsDir))
                        else:
                            if n in neighborsDir:
                                emission[(((n.x,n.y),(x,y)),d)] = 0.6/len(neighborsDir)
                            else:     
                                emission[(((n.x,n.y),(x,y)),d)] = 0.4/(len(neighbors) - len(neighborsDir) + 1)
                    if neighborF != None:
                        emission[(((x,y),(x,y)),d)] = 0.2/(len(neighbors) - len(neighborsDir))
                    else:
                        emission[(((x,y),(x,y)),d)] = 0.4/(len(neighbors) - len(neighborsDir) + 1)
        return trans, emission

    """
    Find neighbors of the current cell that are in the same direction the human is facing
    """
    def neighbors_dir(self, cell, direction):
        neighbors = []
        neighborD = None
        #not on right border
        if cell.x < self.max_width-self.diff and (direction == E or direction == SE or direction == NE):
            if direction == E:
                neighborD = self.cells[(cell.x+self.diff, cell.y)]
            else:
                neighbors.append(self.cells[(cell.x+self.diff, cell.y)])
        #not on right border and bottom border
        if cell.x < self.max_width-self.diff and cell.y > self.min_height and (direction == E or direction == SE or direction == S):
            if direction == SE:
                neighborD = self.cells[(cell.x+self.diff, cell.y-self.diff)]
            else:
                neighbors.append(self.cells[(cell.x+self.diff, cell.y-self.diff)])
        #not on bottom border
        if cell.y > self.min_height and (direction == S or direction == SE or direction == SW):
            if direction == S:
                neighborD = self.cells[(cell.x, cell.y-self.diff)]
            else:
                neighbors.append(self.cells[(cell.x, cell.y-self.diff)])
        #not on left border and bottom border
        if cell.x > self.min_width and cell.y > self.min_height and (direction == W or direction == SW or direction == S):
            if direction == SW:
                neighborD = self.cells[(cell.x-self.diff, cell.y-self.diff)]
            else:
                neighbors.append(self.cells[(cell.x-self.diff, cell.y-self.diff)])
        #not on left border
        if cell.x > self.min_width and (direction == W or direction == SW or direction == NW):
            if direction == W:
                neighborD = self.cells[(cell.x-self.diff, cell.y)]
            else:
                neighbors.append(self.cells[(cell.x-self.diff, cell.y)])
        #not on left border and top border
        if cell.x > self.min_width and cell.y < self.max_height-self.diff and (direction == W or direction == NW or direction == N):
            if direction == NW:
                neighborD = self.cells[(cell.x-self.diff, cell.y+self.diff)]
            else:
                neighbors.append(self.cells[(cell.x-self.diff, cell.y+self.diff)])
        #not on top border
        if cell.y < self.max_height-self.diff and (direction == N or direction == NW or direction == NE):
            if direction == N:
                neighborD = self.cells[(cell.x, cell.y+self.diff)]
            else:
                neighbors.append(self.cells[(cell.x, cell.y+self.diff)])
        #not on right border and top border
        if cell.x < self.max_width-self.diff and cell.y < self.max_height-self.diff and (direction == E or direction == N or direction == NE):
            if direction == NE:
                neighborD = self.cells[(cell.x+self.diff, cell.y+self.diff)]
            else:
                neighbors.append(self.cells[(cell.x+self.diff, cell.y+self.diff)])
        return neighbors, neighborD

    """
    Calculate the probability of the human taking paths that will collide with the
    robot in 2 moves using HMMs
    """
    def pathProbs(self, path, humanPos, humanDir):
        #get possible 2 step paths from human's current position
        #that may collide with robot in 2 steps
        move2 = path[1]
        rneigh = self.get_neighbors(self.cells[move2])
        hneigh = self.get_neighbors(self.cells[humanPos])
        midCells = []
        #reachable in 2 moves
        for n in rneigh:
            if n in hneigh:
                midCells.append(n)
        #reachable by staying once and moving once
        if move2 in hneigh:
            midCells.append(humanPos)
            midCells.append(move2)

        #determine if human likely to take path that may collide
        changeCells = []
        if len(midCells) != 0:
            #calculate prob of going in path using HMMs
            for m in midCells:
                mcoor = (m.x,m.y)
                #transitions
                trans1 = self.initProb
                trans2 = self.trans[(mcoor,humanPos)]
                trans3 = self.trans[(move2,mcoor)]
                #directions = assume facing the direction will be in for future moves
                d1 = humanDir
                d2 = math.atan2(mcoor[1] - humanPos[1], mcoor[0] - humanPos[0]) - humanDir
                d3 = math.atan2(move2[1] - mcoor[1], move2[0] - mcoor[0]) - d2
                degrees1 = round(math.degrees(d1)/45)*45
                degrees2 = round(math.degrees(d2)/45)*45
                degrees3 = round(math.degrees(d3)/45)*45
                if degrees1 == 360:
                    degrees1 = 0
                if degrees2 == 360:
                    degrees2 = 0
                if degrees3 == 360:
                    degrees3 = 0
                d1 = math.radians(degrees1)
                d2 = math.radians(degrees2)
                d3 = math.radians(degrees3)
                if d1 < 0.0:
                    d1 += (math.pi * 2.0)
                if d2 < 0.0:
                    d2 += (math.pi * 2.0)
                if d3 < 0.0:
                    d3 += (math.pi * 2.0)
                #emissions
                em1 = self.emission[((humanPos,humanPos),d1)]
                em2 = self.emission[((mcoor,humanPos),d2)]
                em3 = self.emission[((move2,mcoor),d3)]
                pathProb = trans1*em1+trans2*em2+trans3*em3
                rospy.loginfo("The prob of collision with midCell (" + str(m.x) + "," + str(m.y) + ") is " + str(pathProb))

                #add cell if high prob of collision occurring
                if pathProb >= 0.045:
                    for n in hneigh:
                        changeCells.append((n.x,n.y))

        #adjust costs if needed
        if len(changeCells) > 0:
            if move2 not in changeCells:
                changeCells.append(move2)
            for c in changeCells:
                self.sum_cost[c] += 100.0
                self.changed.append(c)
            return True
        return False
        