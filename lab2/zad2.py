from dimacs import *

(V, L) = loadWeightedGraph("clique5")       

graph = [[0 for _ in range(V)] for _ in range(V)]
for a, b, w in L:
    graph[a-1][b-1] = w




def bfs(G, s, t, parent):
    visited = [False] * len(G)
    queue = []
    queue.append(s)
    visited[s] = True
 
    while queue:
        u = queue.pop(0)
        for ind in range(len(G[u])):
            if visited[ind] is False and G[u][ind] > 0:
                queue.append(ind)
                visited[ind] = True
                parent[ind] = u
 
    return True if visited[t] else False
 
 
def FordFulkerson(G, source, sink):
    parent = [-1] * (len(G))
    max_flow = 0
    while bfs(G, source, sink, parent):
        path_flow = float("Inf")
        s = sink
 
        while s != source:
            path_flow = min(path_flow, G[parent[s]][s])
            s = parent[s]
 
        max_flow += path_flow
        v = sink
 
        while v != source:
            u = parent[v]
            G[u][v] -= path_flow
            G[v][u] += path_flow
            v = parent[v]
    return max_flow
 
 

print("Ford fulkerson:",FordFulkerson(graph, 0, 1))

graph = [[0 for _ in range(V)] for _ in range(V)]
for a, b, w in L:
    graph[a-1][b-1] = w


def max_flow(graph, s, t):
        n = len(graph)
        F = [[0] * n for _ in range(n)]
        path = bfs(graph, F, s, t)
        while path != None:
            flow = min(graph[u][v] - F[u][v] for u,v in path)
            for u,v in path:
                F[u][v] += flow
                F[v][u] -= flow
            path = bfs(graph, F, s, t)
        return sum(F[s][i] for i in range(n))


def bfs(G, F, s, t):
    queue = [s]
    paths = [[] for _ in range(len(G))]
    
    if s == t:
        return paths[s]

    while queue: 
        u = queue.pop(0)
        for v in range(len(G)):
            if (G[u][v] - F[u][v] > 0) and not paths[v]:
                paths[v] = paths[u] + [(u, v)]
                if v == t:
                    return paths[v]
                queue.append(v)
    
    return None



print ("Edmonds Karp:", max_flow(graph, 0, 1))
