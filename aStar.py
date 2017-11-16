import heapq
import math

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
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = set()
        self.cells = {}
        self.grid_height = None
        self.grid_width = None
        self.start = None
        self.end = None
        self.sum_cost = None
        self.sum_avoid = None
        self.human = None
        self.hvel = 0
        self.evidence = []

    """
    Initializes grid robot will travel in and assigns cost
    to each position in grid. Also gets the start position
    and determines the best end position.
    """
    def init_grid(self, width, height, start, goal, theta, human):
        #create cells for grid and get their cost
        self.sum_cost = {}
        self.grid_height = height
        self.grid_width = width
        for x in range(1,self.grid_width):
            for y in range(1,self.grid_height):
                self.cells[(x,y)] = Cell(x, y)
                self.sum_cost[(x,y)] = self.getCost(x, y, goal, theta)
        self.start = self.cells[start]

        #find min cost point for destination
        min_coor = start
        min_cost = self.sum_cost[min_coor]
        for x in range(1,self.grid_width):
            for y in range(1,self.grid_height):
                if self.sum_cost[(x,y)] < min_cost:
                    min_coor = (x,y)
                    min_cost = self.sum_cost[min_coor]
        self.end = self.cells[min_coor]

        #get costs with human obstacle
        self.sum_avoid = dict(self.sum_cost)
        self.human = human
        self.sum_avoid[human] += 100.0
        human_neighbors = self.get_neighbors(self.cells[human])
        for n in human_neighbors:
            self.sum_avoid[(n.x,n.y)] += 80.0

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
        if dist >= 2.0:
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
        if cell.x < self.grid_width-1:
            neighbors.append(self.cells[(cell.x+1, cell.y)])
        #not on right border and bottom border
        if cell.x < self.grid_width-1 and cell.y > 1:
            neighbors.append(self.cells[(cell.x+1, cell.y-1)])
        #not on bottom border
        if cell.y > 1:
            neighbors.append(self.cells[(cell.x, cell.y-1)])
        #not on left border and bottom border
        if cell.x > 1 and cell.y > 1:
            neighbors.append(self.cells[(cell.x-1, cell.y-1)])
        #not on left border
        if cell.x > 1:
            neighbors.append(self.cells[(cell.x-1, cell.y)])
        #not on left border and top border
        if cell.x > 1 and cell.y < self.grid_height-1:
            neighbors.append(self.cells[(cell.x-1, cell.y+1)])
        #not on top border
        if cell.y < self.grid_height-1:
            neighbors.append(self.cells[(cell.x, cell.y+1)])
        #not on right border and top border
        if cell.x < self.grid_width-1 and cell.y < self.grid_height-1:
            neighbors.append(self.cells[(cell.x+1, cell.y+1)])
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

        path.append((self.start.x, self.start.y))
        path.reverse()
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

    def makeProbs(self, pos, ev):
        trans = {}
        revEm = {}

        #only look at neighbors for transitions (max 8 neighbors)
        neighbors = self.get_neighbors(self.cells[pos])

        for n in neighbors:
            #transition between positions
            trans[(n.x,n.y)] = 1.0 / (self.grid_width * self.grid_height)

            #prob of moving to neighbor based on evidence
            angle = math.atan2(n.y - pos[1], n.x - pos[0]) - ev
            #moving in direction facing
            if angle == 0.0:
                revEm[(n.x,n.y)] = 0.5
            else:
                revEm[(n.x,n.y)] = 0.05

        #prob of staying based on evidence
        revEm[pos] = 1 - sum(revEm.values())
