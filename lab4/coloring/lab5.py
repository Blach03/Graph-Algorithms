import heapq
from dimacs import *

(V, L) = loadWeightedGraph("AT")       

graph = [[0 for _ in range(V)] for _ in range(V)]
for a, b, w in L:
    graph[a-1][b-1] = w
    graph[b-1][a-1] = w


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

def chromatic_number(graph):
    peo = lex_bfs(graph)
    color = [0] * len(graph)

    for v in peo:
        neighbors = graph[v]
        used_colors = {color[u] for u in neighbors}
        c = 1

        while c in used_colors:
            c += 1

        color[v] = c

    num_colors = max(color)
    return num_colors, color



chromatic_number_result, coloring = chromatic_number(graph_dict)
print("Liczba chromatyczna grafu:", chromatic_number_result)
