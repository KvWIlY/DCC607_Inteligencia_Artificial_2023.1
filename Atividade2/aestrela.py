import heapq

# Class Graph que representa o grafo
class Graph:
    #Função inicializa um grafo com as conexões e os pesos das arestas
    def __init__(self, connections, weights):
        self.graph = {}
        for i, node in enumerate(connections):
            if node[0] not in self.graph:
                self.graph[node[0]] = {}
            self.graph[node[0]][node[1]] = weights[i]
            if node[1] not in self.graph:
                self.graph[node[1]] = {}
            self.graph[node[1]][node[0]] = weights[i]

    # Função implementa o algoritmo A* para encontrar o caminho mais curto entre dois nós do grafo.
    #  A implementação é baseada em um heap de prioridade para manter o conjunto de nós a serem 
    # explorados em ordem crescente de custo estimado 
    def a_star(self, start, goal):
        frontier = [(0, start)]
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while frontier:
            current_cost, current_node = heapq.heappop(frontier)

            if current_node == goal:
                break

            for neighbor in self.graph[current_node]:
                new_cost = cost_so_far[current_node] + self.graph[current_node][neighbor]
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(goal, neighbor)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current_node

        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path.reverse()
        return path, cost_so_far[goal]

    #Função heurística que usada aqui é uma estimativa da distância restante entre dois nós. 
    def heuristic(self, a, b):
        h = {
            'Arad': 366,
            'Bucharest': 0,
            'Craiova': 160,
            'Dobreta': 242,
            'Eforie': 161,
            'Fagaras': 176,
            'Giurgiu': 77,
            'Hirsova': 151,
            'Iasi': 226,
            'Lugoj': 244,
            'Mehadia': 241,
            'Neamt': 234,
            'Oradea': 380,
            'Pitesti': 100,
            'Rimnicu Vilcea': 193,
            'Sibiu': 253,
            'Timisoara': 329,
            'Urziceni': 80,
            'Vaslui': 199,
            'Zerind': 374
        }
        return h[a] - h[b]

#representam as conexões entre cidades em um mapa
connections = [('Arad', 'Zerind'), ('Zerind', 'Oradea'), ('Oradea', 'Sibiu'), ('Sibiu', 'Fagaras'), 
               ('Fagaras', 'Bucharest'), ('Bucharest', 'Pitesti'), ('Pitesti', 'Rimnicu Vilcea'),
               ('Rimnicu Vilcea', 'Sibiu'), ('Rimnicu Vilcea', 'Craiova'), ('Craiova', 'Dobreta'),
               ('Craiova', 'Pitesti'), ('Bucharest', 'Giurgiu'), ('Bucharest', 'Urziceni'),
               ('Urziceni', 'Hirsova'), ('Hirsova', 'Eforie'), ('Urziceni', 'Vaslui'),
               ('Vaslui', 'Iasi'), ('Iasi', 'Neamt'), ('Arad', 'Timisoara'), ('Timisoara', 'Lugoj'),
               ('Lugoj', 'Mehadia'), ('Mehadia', 'Dobreta')]

#Isso é uma lista de pesos associados a cada conexão na lista connections.
weights = [75, 71, 151, 99, 211, 101, 97, 80, 146, 120, 138, 90, 90, 85, 98, 86, 142, 92, 87, 87, 70, 75, 70, 75]

# processo de criação do grafo com as conecções e os pesos
graph = Graph(connections, weights)
start = 'Arad'
goal = 'Bucharest'

path, cost = graph.a_star(start, goal)

print(' -> '.join(path))
print(f'Total cost: {cost}')

























































#obs: codigo não é 100% autoral
