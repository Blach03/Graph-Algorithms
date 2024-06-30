from dimacs import *

(V, L) = loadWeightedGraph("pp100")       

graf = [[] for _ in range(V)]
for a, b, w in L:
    graf[a-1].append((b-1, w))
    graf[b-1].append((a-1, w))


def find(parent, x):
    if parent[x] != x:
        parent[x] = find(parent, parent[x])
    return parent[x]


def union(parent, rank, x, y):
    root_x = find(parent, x)
    root_y = find(parent, y)
    if root_x != root_y:
        if rank[root_x] > rank[root_y]:
            parent[root_y] = root_x
        else:
            parent[root_x] = root_y
            if rank[root_x] == rank[root_y]:
                rank[root_y] += 1


def sort_edges(G):
    edges = []
    for u, neighbors in enumerate(G):
        for v, weight in neighbors:
            edges.append((u, v, weight))
    return sorted(edges, key=lambda edge: -edge[2])


def find_union(G, s, t):
    n = len(G)
    parent = list(range(n))
    rank = [0] * n
    edges = sort_edges(G)
    max_weight = float('inf')

    for edge in edges:
        u, v, weight = edge
        if find(parent, u) != find(parent, v):
            union(parent, rank, u, v)
            max_weight = min(max_weight, weight)
            if find(parent, s) == find(parent, t):
                return max_weight

    return max_weight


result = find_union(graf, 0, 1)
print("Union:", result) 

def binary_search(G, s, t):
    def dfs(G, u, t, visited, min_weight):
        if u == t:
            return True
        visited[u] = True

        for v, weight in G[u]:
            if not visited[v] and weight >= min_weight:
                if dfs(G, v, t, visited, min_weight):
                    return True
        return False

    min_weight = 0
    max_weight = max(weight for neighbors in G for _, weight in neighbors)
    result = -1

    while min_weight <= max_weight:
        mid_weight = (min_weight + max_weight) // 2
        visited = [False] * len(G)

        if dfs(G, s, t, visited, mid_weight):
            result = mid_weight
            min_weight = mid_weight + 1
        else:
            max_weight = mid_weight - 1

    return result


result = binary_search(graf, 0, 1)
print("Binary search:", result) 



from queue import PriorityQueue

def dijkstra(G, s, t):
    n = len(G)
    parent = [None] * n
    dist = [0] * n
    dist[s] = float("inf")
    q = PriorityQueue()
    q.put((0, s))
    visited = [False] * n

    while not q.empty():
        dst, v = q.get()
        
        if visited[v]:
            continue
        visited[v] = True

        for u, weight in G[v]:
            if not visited[u]:
                new_dist = min(dist[v], weight)
                if new_dist > dist[u]:
                    dist[u] = new_dist
                    parent[u] = v
                    q.put((-dist[u], u))

    return dist[t]


result = dijkstra(graf,0,1)
print("Dijkstra:", result) 
