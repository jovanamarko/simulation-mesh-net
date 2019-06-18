"""
Проект по предметот Безжични мултимедиски системи
Тема: "Симулација на праќање пакети низ mesh мрежа"

Изработила: Јована Марковска 141189
Ментор: Проф. Милош Јовановиќ

ФИНКИ, 2019

"""


import random
import string
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

from time import perf_counter
from time import perf_counter_ns


from collections import deque, namedtuple


# infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
  return Edge(start, end, cost)


class Graph:
    def __init__(self, edges):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(current_vertex)
            if distances[current_vertex] == inf:
                break
            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path


print("Vnesi broj na mesh jazli: ")
a = input()
a = int(a)
print("Vnesi max broj na jazli za sekoj mesh node: ")
n = input()
n = int(n)


vertices = []
edges = []
nodes = dict()

g = nx.Graph()

for i in range(0, int(a)):
    letter = random.choice(string.ascii_lowercase)
    if letter in vertices:
        letter = random.choice(string.ascii_lowercase)
    vertices.append(letter)

print("PRINTING VERTICES:")
print(vertices)

# Adding nodes to mesh nodes and crating the nx.Graph
for i in vertices:
    num_nodes = round(random.uniform(2, n))
    nodes_to_mesh = []
    for j in range(1, num_nodes):
        index = random.choice(string.ascii_letters) + str(round(random.uniform(0, 9))) + str(
            round(random.uniform(0, 9)))

        nodes_to_mesh.append(index)
        nodes[i] = nodes_to_mesh

        g.add_edge(i, index, color='g', weight=2)

print("PRINTING NODES FOR EACH MESH:")
print(nodes)

# Connecting the mesh nodes witch each other in the nx.Graph and adding edges
# Hard-coded connection -- expand with randomness
for ver1 in vertices:

    ver2 = random.choice(vertices)
    ver3 = random.choice(vertices)
    ver4 = random.choice(vertices)

    if ver1 != ver2 and ver1 != ver3 and ver1 != ver4:
        weight1 = round(random.uniform(5, 15))
        weight2 = round(random.uniform(5, 15))
        weight3 = round(random.uniform(5, 15))

        connection1 = tuple((ver1, ver2, weight1))
        connection2 = tuple((ver1, ver3, weight2))
        connection3 = tuple((ver1, ver4, weight3))

        edges.append(connection1)
        edges.append(connection2)
        edges.append(connection3)

        g.add_edge(ver1, ver2, color='b', weight=2)
        g.add_edge(ver1, ver3, color='b', weight=2)
        g.add_edge(ver1, ver4, color='b', weight=2)
    else:
        ver2 = random.choice(vertices)
        ver3 = random.choice(vertices)
        ver4 = random.choice(vertices)

        if ver1 != ver2 and ver1 != ver3 and ver1 != ver4:
            weight1 = round(random.uniform(5, 15))
            weight2 = round(random.uniform(5, 15))
            weight3 = round(random.uniform(5, 15))

            connection1 = tuple((ver1, ver2, weight1))
            connection2 = tuple((ver1, ver3, weight2))
            connection3 = tuple((ver1, ver4, weight3))

            edges.append(connection1)
            edges.append(connection2)
            edges.append(connection3)

            g.add_edge(ver1, ver2, color='b', weight=2)
            g.add_edge(ver1, ver3, color='b', weight=2)
            g.add_edge(ver1, ver4, color='b', weight=2)

rabovi = g.edges()
jazli = g.nodes()
colors = [g[u][v]['color'] for u, v in rabovi]
weights = [g[u][v]['weight'] for u, v in rabovi]
pos = nx.spring_layout(g)

nx.draw(g, pos, edge_color=colors, width=weights, edge_labels=weights, with_labels=True)
# nx.draw_networkx_nodes(g, pos, nodelist=jazli, node_color=colors, width=weights)
plt.show()


# Choosing random start and end node for the packet
s = random.choice(vertices)
arr1 = nodes[s]
sender = random.choice(arr1)
r = random.choice(vertices)
arr2 = nodes[r]     # za vo ista mreza da bide samo smeni r so s
receiver = random.choice(arr2)

print("Sender: " + str(sender) + " from mesh node: " + str(s))
print("Receiver: " + str(receiver) + " from mesh node: " + str(r))

# kolku paketi
num_of_packets = 156000  # avg size of 1Gb video

# Prvo prebaruva vo svojata mreza (simulacija deka mesh jazolot ne znae kako se povrzani ostanatite lokalni mrezi)
tup = nodes[s]
flag = False
for i in tup:
    if i == receiver:
        flag = True

        print("Destinaciskiot jazol e vo istata mreza so prakjachot")
        print("Paketot se prakja...")
        start = perf_counter_ns()
        for i in range(0, num_of_packets):
            continue
        stop = perf_counter_ns()
        timer1 = stop - start
        print("Paketot e praten za " + str(round(timer1)) + " nanosekundi")

if not flag:
    print("Destinaciskiot jazol ne e vo istata mreza so prakjachot")
    print("Se bara destinaciskiot jazol vo mrezata...")
    start = perf_counter()
    for node in nodes:
        for j in nodes[node]:
            if j == receiver:
                print("Destinaciskiot jazol e naden vo mrezata na " + str(node) + " jazol")

    print("Paketot se prakja vo razlichna mreza od prakjachot...")
    graph = Graph(edges)

    # print(graph)
    for i in range(0, num_of_packets):
        graph.dijkstra(s, r)  # the path is calculated on every sending
    stop = perf_counter()
    timer2 = stop - start
    print("Paketot e praten za " + str(round(timer2)) + " sekundi")
    print("Shortest Dijkstra path (node1, node2):")
    print(graph.dijkstra(s, r))

    path = np.asarray(graph.dijkstra(s, r))
    if len(path) == 0:
        print("Pronajdeni se 2 isti najkratki pateki!")

    boi = []
    boi_edge = []

    for node in g.nodes:
        if node in path:
            boi.append('r')
        else:
            boi.append('g')

    # the following code is for coloring the edges in red that are part of the path
    pairs = []
    for i in path:
        k = list(path).index(i)
        if k != len(path) - 1:
            j = path[k + 1]
            pairs.append((i, j))
    print(pairs)
    for edge in g.edges:
        if edge in pairs or (edge[1], edge[0]) in pairs:
            boi_edge.append('r')
        else:
            boi_edge.append('b')

    nx.draw(g, pos, width=2, node_color=boi, edge_color=boi_edge, with_labels=True)
    plt.show()

"""
References:
https://dev.to/mxl/dijkstras-algorithm-in-python-algorithms-for-beginners-dkc
https://gist.github.com/hayderimran7/09960ca438a65a9bd10d0254b792f48f
https://www.reddit.com/r/askscience/comments/41sgzw/how_much_work_does_it_take_to_send_a_packet/
"""