import heapq
from dimacs import *

(V, L) = loadWeightedGraph("clique5")       

graph = [[0 for _ in range(V)] for _ in range(V)]
for a, b, w in L:
    graph[a-1][b-1] = w
    graph[b-1][a-1] = w


graph_dict = {}
n = len(graph)

for i in range(n):
    neighbors = [j for j in range(n) if graph[i][j] == 1]
    graph_dict[i] = neighbors
    


from collections import deque

def lex_bfs(graph, start):
    n = len(graph)
    lex_order = []
    remaining_vertices = set(range(n))

    current_set = remaining_vertices.copy()
    current_set.remove(start)

    while current_set:
        v = min(current_set)
        lex_order.append(v)

        next_set = set()

        for u in current_set:
            if u in graph[v]:
                next_set.add(u)

        current_set = next_set
  
    return lex_order

def is_chordal(graph):
    n = len(graph)

    for start_vertex in range(n):
        lex_order = lex_bfs(graph, start_vertex)
        n=len(lex_order)
        
        for i in range(n):
            v = lex_order[i]
            rn_v = set(graph[v]) & set(lex_order[:i])
            parent_v = max(rn_v, key=lambda x: lex_order.index(x), default=None)
            
            if not set(rn_v - {parent_v}).issubset(set(graph.get(parent_v, []))):
                return False

    return True



result = is_chordal(graph_dict)
print("Czy graf jest przekątniowy?", result)
