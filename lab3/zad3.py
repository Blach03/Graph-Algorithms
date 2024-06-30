from dimacs import *
from collections import defaultdict

name = 'rand20_100'


class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = defaultdict(list)

    def add_edge(self, u, v):
        self.graph[u].append(v)
        self.graph[v].append(u)

    def ford_fulkerson(self, source, sink):
        max_flow = 0

        while self.bfs(source, sink):
            path_flow = float('inf')
            s = sink

            while s != source:
                path_flow = min(path_flow, self.parent[s][1])
                s = self.parent[s][0]

            max_flow += path_flow
            v = sink

            while v != source:
                u = self.parent[v][0]
                self.graph[u].remove(v)
                self.graph[v].remove(u)
                v = u

        return max_flow

    def bfs(self, source, sink):
        visited = [False] * self.V
        queue = []
        self.parent = {}

        queue.append(source)
        visited[source] = True

        while queue:
            u = queue.pop(0)

            for v in self.graph[u]:
                if not visited[v]:
                    queue.append(v)
                    visited[v] = True
                    self.parent[v] = (u, 1)

                    if v == sink:
                        return True

        return False

def edge_connectivity(graph):
    source = 0
    sink = 1

    g = Graph(len(graph))

    for i in range(len(graph)):
        for j in range(len(graph[i])):
            if graph[i][j] == 1:
                g.add_edge(i, j)

    return g.ford_fulkerson(source, sink)


(V, L) = loadWeightedGraph(name)      
graph = [[0 for _ in range(V)] for _ in range(V)]
for a, b, w in L:
    graph[a-1][b-1]=w


result = edge_connectivity(graph)
print("Furkelson:", result)




from dimacs import *
from sys import maxsize
from queue import PriorityQueue

class Node:
    def __init__(self, i):
        self.edges = {}
        self.vertices = [i]

    def addEdge(self, to, weight):
        self.edges[to] = self.edges.get(to, 0) + weight

    def delEdge(self, to):
        del self.edges[to]

def buildGraph(V, E):
    G = [Node(i) for i in range(V)]
    for (x, y, c) in E:
        G[x].addEdge(y, c)
        G[y].addEdge(x, c)
    return G

def verticesNumber(G):
    V = len(G)
    result = 0
    for i in range(1, V):
        if G[i].vertices:
            result += 1

    return result

(V, E) = loadWeightedGraph(name)
G = buildGraph(V+1, E)

def merge(G, i, j):
    for (v, c) in G[j].edges.items():
        G[i].vertices += G[j].vertices
        G[j].vertices = []
        if v == i:
            G[i].delEdge(j)
        else:
            c = G[v].edges.get(j)
            G[v].delEdge(j)
            G[v].addEdge(i, c)
            G[i].addEdge(v, c)


def minimumCutPhase(G, a = 1):
    V = len(G)
    u = None
    v = a
    S = [a]
    edges_sum = [0] * V

    visited = [False] * V
    Q = PriorityQueue()
    Q.put((-edges_sum[a], a))

    while not Q.empty():
        u = v
        (k, v) = Q.get()
        if not visited[v]:
            s = u
            t = v
            visited[v] = True
            for (y, c) in G[v].edges.items():
                if not visited[y]:
                    edges_sum[y] += c
                    Q.put((-edges_sum[y], y))

    return (s, t, edges_sum[t])

def minimumCut(G):
    result = maxsize
    while verticesNumber(G) > 1:
        (s, t, c) = minimumCutPhase(G, 1)
        if c < result:
            result = c
        merge(G, s, t)
    return result


print("Stoer wagner", minimumCut(G))