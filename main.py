import csv
import math
import time
from collections import deque
import heapq


# video link: https://umsystem.hosted.panopto.com/Panopto/Pages/Viewer.aspx?id=6dce1e53-6097-4639-a700-b209014e5c99

"""
Location class to represent each town, which stores the town’s name, 
coordinates, and its adjacent locations
"""
class Location():
    def __init__(self, name, x, y):
        self.name = name
        self.x = float(x) # latitude of the location
        self.y = float(y) # longitude of the location
        self.adj = []  # list to store adjacent location
        self.visited = False

    def __lt__(self, other):
        return False

# Display the sequence of towns 
def RouteString(routes):
    return " -> ".join([route.name for route in routes])



"""
 Haversine formula to calculate distance between two points on the Earth
# Code modified from https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points

"""
def haversine(loc1, loc2):
    ola, olo = math.radians(loc1.x), math.radians(loc1.y)
    ela, elo = math.radians(loc2.x), math.radians(loc2.y)
    lad, lod = ela - ola, elo - olo
    a = math.sin(lad / 2) ** 2 + math.cos(ola) * math.cos(ela) * math.sin(lod / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Radius of Earth in km
    return r * c * 0.621371  # Convert km to miles

#  Function to calculate the total distance of a given route
def RouteDistance(route):
    prev = None
    distance = 0
    for r in route:
        if prev is None:
            prev = r
            continue
        distance += haversine(prev, r) # Calculate the distance
        prev = r
    return round(distance, 2)    

"""
# Code modified from https://favtutor.com/blogs/breadth-first-search-python
Breadth first search to find the shortest path 
"""
def BreadthFirst(start, end):
    visited = [] # List to keep track of visited nodes
    queue = [] # Queue to store paths to be explored
    visited.append(start)
    queue.append([start])

    while queue:
        path = queue.pop(0) # Get the first path from the queue
        node = path[-1] # Get the last node from the path

        if node == end:
            return path # return the path if the end node is found
        
        for neighbour in node.adj:
            if neighbour not in visited:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path) # Add new path to the queue
                visited.append(neighbour) # Mark the node as visited
    return None

"""
# Code modified from https://favtutor.com/blogs/depth-first-search-python

Depth first search to find a path from start to end
"""
def DepthFirst(start, end):
    visited = set()
    stack = [[start]]

    while stack:
        path = stack.pop() # Get the last path from the stack
        node = path[-1] # Get the last node from the path

        if node not in visited:
            visited.add(node)
            if node == end:
                return path  # Return the path if the end node is found
            # Explore all possible paths, then add new path to the stack
            for neighbour in reversed(node.adj):
                new_path = list(path)
                new_path.append(neighbour)
                stack.append(new_path)
    return None

"""
Modified from ChatGPT 4.0
Prompt: You are given a list of cities and a list of city adjacency pairs. Please use undirected (blind) brute-force approaches to write a IDDFS function.
"""
def IDDFS(start, end, max_depth=10):
    for depth in range(max_depth):
        for location in location_dict.values():
            location.visited = False
        path = []
        result = DLS(start, end, depth, path)
        if result:
            return result
    return None
"""
The IDDFS method uses DLS to iterative through increasing depths until a solution is found.
"""
def DLS(start, end, depth, path):
    path.append(start)
    start.visited = True
    if start == end:
        return path
    if depth <= 0:
        path.pop()
        return None
    for neighbor in start.adj:
        if not neighbor.visited:
            result = DLS(neighbor, end, depth - 1, path)
            if result:
                return result
    path.pop()
    return None

"""
Modified from ChatGPT 4.0
Prompt: You are given a list of cities and a list of city adjacency pairs. Please use heuristic approaches to write a best first search function.
"""
def BestFirst(start, end):
    for location in location_dict.values():
        location.visited = False
    pq = [(0, start, [start])]
    while pq:
        _, current, path = heapq.heappop(pq)
        if current.visited:
            continue
        current.visited = True
        if current == end:
            return path
        for neighbor in current.adj:
            if not neighbor.visited:
                new_path = list(path)
                new_path.append(neighbor)
                priority = haversine(neighbor, end)
                heapq.heappush(pq, (priority, neighbor, new_path))
    return None

"""
Modified from ChatGPT 4.0
Prompt: You are given a list of cities and a list of city adjacency pairs. Please use heuristic approaches to write a A* search function.
"""
def AStar(start, end):
    for location in location_dict.values():
        location.visited = False
    pq = [(0, 0, start, [start])]
    while pq:
        cost, _, current, path = heapq.heappop(pq)
        if current.visited:
            continue
        current.visited = True
        if current == end:
            return path
        for neighbor in current.adj:
            if not neighbor.visited:
                new_path = list(path)
                new_path.append(neighbor)
                g = cost + haversine(current, neighbor) #path cost 
                h = haversine(neighbor, end) #heuristic distance 
                f = g + h
                heapq.heappush(pq, (f, g, neighbor, new_path))
    return None

# Dictionary to store all locations by name
location_dict = {}

# Read location from CSV file and create location objects
with open("coordinates.csv", newline='') as f:
    reader = csv.reader(f, delimiter=',', quotechar='|')
    for r in reader:
        l = Location(name=r[0], x=r[1].lstrip(), y=r[2].lstrip())
        location_dict[l.name] = l

# Read adjacency information from text file and update adjacency lists
with open("Adjacencies.txt", "r") as adj_file:
    lines = adj_file.readlines()
    for l in lines:
        l = l.strip()
        aj = l.split(" ")
        a = location_dict.get(aj[0])
        b = location_dict.get(aj[1])
        if a and b:
            a.adj.append(b)
            b.adj.append(a)

# Loop to interact with user and find paths between towns
while True:
    start_name = input("Input Starting Town: ")
    end_name = input("Input Ending Town: ")
    start = location_dict.get(start_name)
    end = location_dict.get(end_name)
    if not start or not end:
        print("Invalid Starting or Ending Town.")
        continue

    while True:
        # Options can select search method or change location        
        method = input("1: Breadth-First\n2: Depth-First\n3: ID-DFS\n4: Best-First\n5: A*\n6: Change Locations\n\nSelect Option: ")
        try:
            method = int(method)
        except ValueError:
            print("Invalid method selection.")
            continue

        start_time = time.perf_counter()
        if method == 1:
            path = BreadthFirst(start, end)
            if path:
                print(f"Breadth-First found path: {RouteString(path)} with distance {RouteDistance(path)} miles.")
            else:
                print("No path found.")
        elif method == 2:
            for location in location_dict.values():
                location.visited = False
            path = DepthFirst(start, end)
            if path:
                print(f"Depth-First found path: {RouteString(path)} with distance {RouteDistance(path)} miles.")
            else:
                print("No path found.")
        elif method == 3:
            path = IDDFS(start, end)
            if path:
                print(f"ID-DFS found path: {RouteString(path)} with distance {RouteDistance(path)} miles.")
            else:
                print("No path found.")
        elif method == 4:
            path = BestFirst(start, end)
            if path:
                print(f"Best-First found path: {RouteString(path)} with distance {RouteDistance(path)} miles.")
            else:
                print("No path found.")
        elif method == 5:
            path = AStar(start, end)
            if path:
                print(f"A* found path: {RouteString(path)} with distance {RouteDistance(path)} miles.")
            else:
                print("No path found.")
        elif method == 6:
            break # change locations
        else:
            print("Invalid method selection.")
            continue

        end_time = time.perf_counter() # Record the end time
        print(f"Time taken: {end_time - start_time:.4f} seconds\n")

        # Ask user if they want to try another method with the same start/end locations or different locations
        retry = input("Do you want to try another method or location? (y/n): ").strip().lower()
        print("\n")
        # Exit the app if the input is no
        if retry != 'y': 
            print("Thank you for using the App!")
            exit()

   