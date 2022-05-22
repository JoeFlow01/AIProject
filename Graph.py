from collections import deque


class Node:
    def __init__(self, name, weight):
        self.name = name
        self.weight = weight


class Vertex:
    def __init__(self, name):
        self.name = name
        self.Neighbour = []
        self.visited = 0
        self.cost = 0
        self.heuristic = 0
        self.predecessor = None


class Graph:
    explored = []

    def __init__(self):
        self.ListofVertices = []

    def insertVertex(self, name, hValue=0):
        for i in self.ListofVertices:
            if i.name == name:
                return -1
        v = Vertex(name)
        v.heuristic = hValue
        self.ListofVertices.append(v)
        return 1

    def iterative(self, graph, start, goal):
        print("hi")
        limit = 0
        while True:
            visited, path = self.DLS(graph, start, goal, limit)
            if len(path) == 0:
                limit += 1
                print("LIMIT ISSSSSSSSSSSS")
                print(limit)
            else:
                print(path)
                print(self.cost)
                return path, self.cost
    def depth_limited_search(self, start, setOfGoals=None, limit=3):
        print('Depth limit =', limit)
        found, fringe, visited, came_from = False, deque([(0, start)]), set([start]), {start: None}
        print('{:11s} | {}'.format('Expand Node', 'Fringe'))
        print('--------------------')
        print('{:11s} | {}'.format('-', start))
        while (not (found)) and len(fringe):
            depth, current = fringe.pop()
            print('{:11s}'.format(current), depth, end=' | ')
            if current in setOfGoals:
                found = True
                goal = current
                break
            if limit == 3 or depth < limit:
                for node in self.neighbors(current):
                    if node not in visited:
                        visited.add(node)
                        fringe.append((depth + 1, node))
                        came_from[node] = current
            print(', '.join([n for _, n in fringe]))
            print(len(fringe))
        if found:
            print()
            return came_from, visited
        else:
            print('No path from {} to {}'.format(start, setOfGoals))
            return None, visited

    def iterative_deepening_dfs(self, start, setOfGoals=None):
        prev_iter_visited, depth, traced_path = [], 0, {start: None}
        while True:
            traced_path, visited = self.depth_limited_search(start, setOfGoals, depth)
            if traced_path or len(visited) == len(prev_iter_visited):
                goal = list(traced_path.keys())[-1]
                return traced_path, goal
            else:
                prev_iter_visited = visited
                depth += 1
    def insertEdgeDirected(self, v1, v2, weight=0):
        found2 = False
        x = None
        for i in self.ListofVertices:
            if i.name == v2:
                found2 = True
            if i.name == v1:
                x = i
                for j in i.Neighbour:
                    if j.name == v2:
                        return "Exist"

        if found2 is True and x is not None:
            nodev2 = Node(v2, weight)
            x.Neighbour.append(nodev2)
            return "Done"

        elif found2 is False:
            return "V2 ERR"
        return "V1 ERR"

    def insertEdge_unDirected(self, v1, v2, weight=0):
        self.insertEdgeDirected(v1, v2, weight)
        self.insertEdgeDirected(v2, v1, weight)

    def deleteEdge_directed(self, v1, v2):
        for i in self.ListofVertices:
            if i.name == v1:
                for j in i.Neighbour:
                    if j.name == v2:
                        i.Neighbour.remove(j)
                        return "deleted"
                return "edge doesn't exist"
        return "v doesn't exist"

    def deleteEdge_undirected(self, v1, v2):
        self.deleteEdge_directed(v1,v2)
        self.deleteEdge_directed(v2, v1)


    def deleteVertex(self, v):
        found = False
        x = None
        for i in self.ListofVertices:
            if i.name == v:
                x = i
                found = True
            for j in i.Neighbour:
                if j.name == v:
                    i.Neighbour.remove(j)
        if found:
            self.ListofVertices.remove(x)
            return "Done"
        else:
            return "Vertex Doesn't Exist"

    def depthlimited(self, start, goal,limit):#bta3 joe
        stack = []
        x = self.search(start)
        stack.insert(0, (x.name, None))

        while len(stack) != 0:
            x = self.search(stack[0][0])
            x.predecessor = stack[0][1]
            self.explored.append(x.name)
            x.visited = 1
            stack.pop(0)

            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                if y.visited == 0:
                    stack.insert(0, (y.name, x.name))

    def DLS(self, graph, start, goal, maxDepth):
        stack = [start]
        visited = []
        parent = {}
        path = []
        level = 0
        print(graph.keys())
        for node in graph.keys():
            parent[node] = None
        while stack:
            if (level > maxDepth):
                return visited, path, level
            print(stack)
            node = stack.pop(0)
            visited.append(node)
            print("ME")
            print(visited)
            print(stack)
            print("abcsd")
            print(node)
            # print(goal[0])
            if node == goal[0]:
                print("358")
                p = parent[goal[0]]
                path.append(goal[0])
                print("122")
                while p != None:
                    print(p)
                    path.append(p)
                    p = parent[p]
                print("123568")
                path.reverse()
                print(parent)
                print(path)
                print(visited)
                return visited, path
            print("msh goalll")
            children = graph[node]
            print(children)
            print("ay haga")
            for child in children:
                print("sdfaaaab")
                print(child)
                print("4444444444444")
                print(child[0])
                if child[0] not in visited:
                    print(node)
                    parent[child[0]] = node
                    stack.insert(0, child[0])
                    print("289")
                    print(stack)
            if (stack[0], 0) in children:
                level = level + 1
                print("LEVEL IS")
    def DFS(self, start, goal):
        stack = []
        x = self.search(start)
        stack.insert(0, (x.name, None))
        while len(stack) != 0 :
            x = self.search(stack[0][0])
            x.predecessor = stack[0][1]
            self.explored.append(x.name)
            x.visited = 1
            stack.pop(0)

            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                if y.visited == 0:
                    stack.insert(0, (y.name, x.name))

    def DFS_Limited(self, start, goal,limit):

        stack = []
        x = self.search(start)
        stack.insert(0, (x.name, None))
        #print("backend")
        depth=0
        #print("ABL EL while")
        print(depth)
        print(limit)
        while (depth<(limit+1)) : #and (depth <= limit)////len(stack) != 0 and 
            print("D5alna el while")
            x = self.search(stack[0][0])
            x.predecessor = stack[0][1]
            self.explored.append(x.name)
            x.visited = 1
            stack.pop(0)
            #print(depth)
            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                if y.visited == 0:
                    stack.insert(0, (y.name, x.name))
            depth=depth+1
    #
    # import problem

    # def depthLimited(graph, start, goal, depth, stack, visited=None, robot=None):
    #     stack.append(start)
    #
    #     if start == goal:
    #         return stack
    #     elif depth == 0:
    #         stack.pop()
    #         return None
    #
    #     if visited is None:
    #         visited = []
    #
    #     visited.append(start)
    #     for next in graph.get(start):
    #         if type(next) == tuple:
    #             if (not next[0] in visited) and (graph.problem.checkAvailable(graph, next[0], start, robot)):
    #                 if robot != None:
    #                     direction = graph.problem.whichDirection(start, next[0])
    #                     tmp = graph[start][0]
    #                     graph[start][0] = 'x'
    #                     checkDeadlock = graph.problem.isDeadlock(start, robot, "ids", direction, graph)
    #                     graph[start][0] = tmp
    #                     if not checkDeadlock:
    #                         reachGoal = graph.depthLimited(graph, next[0], goal, depth - 1, stack, visited, robot)
    #                         if reachGoal:
    #                             return reachGoal
    #                 else:
    #                     reachGoal = graph.depthLimited(graph, next[0], goal, depth - 1, stack, visited, robot)
    #                     if reachGoal:
    #                         return reachGoal
    #
    #     depth += 1
    #     stack.pop()
    #     visited.pop()
    #
    # def iterativeDeepening(graph, start, goal, length, robot=None):
    #     depth = 0
    #     result = None
    #     while depth < length:
    #         stack = []
    #         result = depthLimited(graph, start, goal, depth, stack, robot=robot)
    #         if result != None:
    #             break
    #         depth += 1
    #     return result, depth
    #

    def Iterative_Deepening(self, start, goal):
        Queue = []
        x = self.search(start)
        Queue.append((x.name, None))
        while len(Queue) != 0:
            x = self.search(Queue[0][0])

            if x.predecessor is None:
                x.predecessor = Queue[0][1]
            x.visited = 1
            self.explored.append(x.name)
            Queue.pop(0)

            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                if y.visited == 0:
                    Queue.append((y.name, x.name))

    def BFS(self, start, goal):
        Queue = []
        x = self.search(start)
        Queue.append((x.name, None))
        while len(Queue) != 0:
            x = self.search(Queue[0][0])

            if x.predecessor is None:
                x.predecessor = Queue[0][1]
            x.visited = 1
            self.explored.append(x.name)
            Queue.pop(0)

            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                if y.visited == 0:
                    Queue.append((y.name, x.name))

    def UCS(self, start, goal):
        priorityQ = []
        x = self.search(start)
        priorityQ.append((x.name, 0))

        while len(priorityQ) != 0:
            x = self.search(priorityQ[0][0])
            self.explored.append(x.name)
            x.visited = 1
            priorityQ.pop(0)

            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                z = x.cost + int(j.weight)
                if y.visited == 0:
                    if y.cost == 0:
                        y.cost = z
                        # priorityQ.append((j.name, y.cost))
                        y.predecessor = x.name
                    elif y.cost > z:
                        y.cost = z
                        # priorityQ.append((j.name, y.cost))
                        y.predecessor = x.name
                    priorityQ.append((j.name, z))
                priorityQ.sort(key=lambda x: x[1])

    def greedy(self, start, goal):
        priorityQ = []
        x = self.search(start)
        priorityQ.append((x.name, 0))

        while len(priorityQ) != 0:

            x = self.search(priorityQ[0][0])
            self.explored.append(x.name)
            x.visited = 1
            priorityQ.pop(0)

            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                if y.visited == 0:
                    priorityQ.append((j.name, y.heuristic))
                    y.predecessor = x.name
            priorityQ.sort(key=lambda x: x[1])

    def aStar(self, start, goal):
        priorityQ = []
        x = self.search(start)
        priorityQ.append((x.name, 0))

        while len(priorityQ) != 0:
            x = self.search(priorityQ[0][0])
            self.explored.append(x.name)
            x.visited = 1
            priorityQ.pop(0)

            if x.name in goal:
                return self.path(x.name)

            for j in x.Neighbour:
                y = self.search(j.name)
                g = x.cost + int(j.weight)
                z = g + y.heuristic
                if y.visited == 0:
                    if y.cost == 0:
                        y.cost = g
                        # priorityQ.append((j.name, z))
                        y.predecessor = x.name
                    elif y.cost > g:
                        y.cost = g
                        # priorityQ.append((j.name, z))
                        y.predecessor = x.name
                    priorityQ.append((j.name, z))
                priorityQ.sort(key=lambda x: x[1])

    def remove_pref(self):
        for i in self.ListofVertices:
            i.predecessor = None
            i.visited = 0
            i.cost = 0

    def path(self, goal):
        path = []
        x = self.search(goal)
        path.append(x.name)
        cost = 0

        while x.predecessor is not None:
            for i in self.search(x.predecessor).Neighbour:
                if i.name == x.name:
                    cost += int(i.weight)
            path.append(x.predecessor)
            x = self.search(x.predecessor)
        path.reverse()

        return path, cost

    def search(self, start):
        for i in self.ListofVertices:
            if i.name == start:
                return i
