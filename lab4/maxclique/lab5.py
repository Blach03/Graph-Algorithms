import heapq
from dimacs import *

(V, L) = loadWeightedGraph("simple")       

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

def maximum_clique_size(graph):
    peo = lex_bfs(graph)
    max_clique_size = 0

    for v in peo:
        neighbors = set(graph[v]) | {v}
        max_clique_size = max(max_clique_size, len(neighbors))

    return max_clique_size


clique_size = maximum_clique_size(graph_dict)
print("Rozmiar największej kliki:", clique_size)