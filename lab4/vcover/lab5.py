import heapq
from dimacs import *

(V, L) = loadWeightedGraph("house")       

graph = [[0 for _ in range(V)] for _ in range(V)]
for a, b, w in L:
    graph[a-1][b-1] = w

graph_dict = {}
n = len(graph)

for i in range(n):
    neighbors = [j for j in range(n) if graph[i][j] == 1]
    graph_dict[i] = neighbors


def lex_bfs(graph):
    n = len(graph)
    lex_order = []
    remaining_vertices = set(range(n))

    current_set = remaining_vertices.copy()

    while current_set:
        v = min(current_set)
        lex_order.append(v)

        next_set = set()

        for u in current_set:
            if u in graph[v]:
                next_set.add(u)

        current_set = next_set

    return lex_order

def minimum_vertex_cover(graph):
    peo = lex_bfs(graph)
    vertex_cover = set()

    for v in reversed(peo):
        neighbors = set(graph[v])

        if not neighbors.intersection(vertex_cover):
            vertex_cover.add(v)

    return vertex_cover, len(vertex_cover)



vertex_cover_set, vertex_cover_size = minimum_vertex_cover(graph_dict)
print("Najmniejsze pokrycie wierzchołkowe:", vertex_cover_set)
