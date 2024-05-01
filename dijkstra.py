MAX_VALUE = 9999 # big number symbolising infinite


class Graph:
    def __init__(self, nodes: list[str]) -> None:
        self.nodes = nodes
        self.connections = dict()
        for node in self.nodes:
            self.connections.update({node: dict((n, MAX_VALUE) for n in self.nodes if n != node)})
    
    def add_connection(self, node1, node2, distance) -> None:
        """Add connection between two nodes"""
        self.connections[node1][node2] = distance
        self.connections[node2][node1] = distance
    
    def get_distance(self, node1, node2) -> float | int:
        """get connection between two nodes. If no connection, return MAX_VALUE"""
        return self.connections[node1][node2]


class Navigation:
    def __init__(self, graph: Graph) -> None:
        self.graph = graph
        self.unvisited_nodes=set()
        self.distances=dict()
        self.prev=dict()

    def dijkstra(self, source: str, target: str) -> None:
        for node in self.graph.nodes:
            self.distances[node] = MAX_VALUE
            self.prev[node] = None
            self.unvisited_nodes.add(node)
        self.distances[source] = 0

        while self.unvisited_nodes:
            closest_unvisited_node = lambda: min((self.distances[n], n) for n in self.unvisited_nodes)[1]
            current_node: str = closest_unvisited_node
            if current_node == target:
                break
            self.unvisited_nodes.remove(current_node)
            is_neighbor = lambda node: self.graph.get_distance(current_node, node) < MAX_VALUE
            neighbors = [neighbor for neighbor in self.unvisited_nodes if is_neighbor(neighbor)]
            for neighbor in neighbors:
                new_distance = self.distances[current_node] + self.graph.get_distance(current_node, neighbor)
                if new_distance < self.distances[neighbor]:
                    self.distances[neighbor] = new_distance
                    self.prev[neighbor] = current_node
    
    def get_path(self, source: str, target: str) -> list[str]:
        self.dijkstra(source, target)
        path = []
        u = target
        if self.prev[u] or source == u:
            while u:
                path.append(u)
                u = self.prev[u]
        path.reverse()
        return path


if __name__ == "__main__":
    # test graph
    graph = Graph(nodes=["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N"])
    graph.add_connection("A", "B", 1)
    graph.add_connection("A", "C", 5)
    graph.add_connection("B", "C", 3)
    graph.add_connection("B", "D", 7)
    graph.add_connection("C", "D", 12)
    graph.add_connection("C", "E", 9)
    graph.add_connection("D", "F", 8)
    graph.add_connection("D", "H", 4)
    graph.add_connection("E", "H", 6)
    graph.add_connection("E", "G", 11)
    graph.add_connection("F", "H", 14)
    graph.add_connection("F", "J", 10)
    graph.add_connection("G", "I", 9)
    graph.add_connection("H", "I", 2)
    graph.add_connection("H", "J", 13)
    graph.add_connection("H", "K", 7)
    graph.add_connection("I", "K", 8)
    graph.add_connection("I", "L", 1)
    graph.add_connection("J", "K", 15)
    graph.add_connection("K", "M", 3)
    graph.add_connection("L", "N", 5)

    navigation = Navigation(graph)
    print(navigation.get_path("A", "G"))