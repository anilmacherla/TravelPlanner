from typing import List
import tkinter as tk
from tkinter import messagebox
from typing import Tuple
from collections import defaultdict
import heapq
import sys


# Define a Graph class to represent the travel network:

class Graph:
    def __init__(self):
        self.edges = defaultdict(list)
        self.vertices = set()

    def add_node(self, node):
        self.vertices.add(node)

    def add_edge(self, from_node: str, to_node: str, distance: float):
        self.edges[from_node].append((to_node, distance))
        self.vertices.add(from_node)
        self.vertices.add(to_node)

    def shortest_path(self, start_node: str, end_node: str) -> Tuple[List[str], float]:
        heap = [(0, start_node)]
        visited = set()
        distances = {start_node: 0}
        previous_nodes = {}
        while heap:
            (current_distance, current_node) = heapq.heappop(heap)
            if current_node in visited:
                continue
            visited.add(current_node)
            if current_node == end_node:
                path = []
                while current_node in previous_nodes:
                    path.append(current_node)
                    current_node = previous_nodes[current_node]
                path.append(start_node)
                path.reverse()
                return path, distances[end_node]
            for (neighbour, distance) in self.edges[current_node]:
                new_distance = current_distance + distance
                if new_distance < distances.get(neighbour, sys.maxsize):
                    distances[neighbour] = new_distance
                    previous_nodes[neighbour] = current_node
                    heapq.heappush(heap, (new_distance, neighbour))
        return [], sys.maxsize


class TravelPlanner:
    """
    A class representing a travel planner.

    Attributes:
        graph (Graph): An instance of the Graph class to represent the travel network.

    """

    def __init__(self):
        """
        Initializes a new instance of the TravelPlanner class.
        """
        self.graph = Graph()

    def add_route(self, from_node: str, to_node: str, distance: float) -> None:
        """
        Adds a new route to the travel network.

        Args:
            from_node (str): The starting node of the route.
            to_node (str): The ending node of the route.
            distance (float): The distance between the two nodes.

        Returns:
            None
        """
        self.graph.add_edge(from_node, to_node, distance)

    def get_shortest_path(self, from_node: str, to_node: str) -> str:
        """
        Finds the shortest path between two nodes in the travel network.

        Args:
            from_node (str): The starting node of the path.
            to_node (str): The ending node of the path.

        Returns:
            str: A string representing the shortest path between the two nodes.
        """
        path, distance = self.graph.shortest_path(from_node, to_node)
        if distance == sys.maxsize:
            return f"No route found from {from_node} to {to_node}."
        formatted_path = " -> ".join(path)
        return f"Shortest route from {from_node} to {to_node} is {distance:.2f} km: {formatted_path}"

    def get_shortest_path_between_cities(self, cities: List[str]) -> str:
        """
        Finds the shortest path between multiple cities in the travel network.

        Args:
            cities (List[str]): A list of cities to find the shortest path between.

        Returns:
            str: A string representing the shortest path between the given cities.
        """
        total_distance = 0
        for i in range(len(cities) - 1):
            distance = self.graph.shortest_path(cities[i], cities[i + 1])[1]
            total_distance += distance
        formatted_cities = " -> ".join(cities)
        return f"Shortest route between {formatted_cities} is {total_distance:.2f} km."


class TravelPlannerGUI:
    """
    A GUI for planning travel routes between cities using Dijkstra's algorithm.

    Attributes:
        master (tk.Tk): The root window of the application.
        graph (Graph): The graph representing the connections between cities.

    """

    def __init__(self, master):
        """
        Initializes the TravelPlannerGUI object.

        Args:
            master (tk.Tk): The root window of the application.

        """

        self.master = master
        master.title("Travel Planner")

        # Create the graph
        self.graph = Graph()
        self.graph.add_node('New York')
        self.graph.add_node('Chicago')
        self.graph.add_node('San Francisco')
        self.graph.add_node('Los Angeles')
        self.graph.add_node('Seattle')
        self.graph.add_edge('New York', 'Chicago', 719)
        self.graph.add_edge('New York', 'San Francisco', 2909)
        self.graph.add_edge('Chicago', 'Los Angeles', 2015)
        self.graph.add_edge('San Francisco', 'Seattle', 808)
        self.graph.add_edge('Los Angeles', 'Seattle', 1131)

        # Create the widgets
        self.label_from = tk.Label(master, text="From:")
        self.label_from.grid(row=0, column=0)

        self.entry_from = tk.Entry(master)
        self.entry_from.grid(row=0, column=1)

        self.label_to = tk.Label(master, text="To:")
        self.label_to.grid(row=1, column=0)

        self.entry_to = tk.Entry(master)
        self.entry_to.grid(row=1, column=1)

        self.button_plan = tk.Button(
            master, text="Plan Trip", command=self.plan_trip)
        self.button_plan.grid(row=2, column=0, columnspan=2)

        self.label_result = tk.Label(master, text="")
        self.label_result.grid(row=3, column=0, columnspan=2)

    def plan_trip(self):
        """
        Plans a trip between two cities based on user input.

        Gets the user input for the starting city and the destination city, then
        uses Dijkstra's algorithm to find the shortest path between the two cities.
        If a path is found, the path and the total distance are displayed in the
        GUI. If no path is found, an error message is displayed.

        """

        # Get the user inputs
        start = self.entry_from.get().strip()
        end = self.entry_to.get().strip()

        # Use Dijkstra's algorithm to find the shortest path
        path, distance = self.graph.shortest_path(start, end)

        # Display the results
        if not path:
            messagebox.showerror(
                "Error", f"No path found from {start} to {end}")
        else:
            result_str = f"Shortest path from {start} to {end}: {path}\n"
            result_str += f"Total distance: {distance} miles"
            self.label_result.configure(text=result_str)


if __name__ == "__main__":
    # Create the main window
    root = tk.Tk()

    # Create the travel planner GUI
    gui = TravelPlannerGUI(root)

    # Start the GUI event loop
    root.mainloop()
