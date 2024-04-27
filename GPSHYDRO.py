import os
import osmnx as ox
import networkx as nx
import heapq
import matplotlib.pyplot as plt
import logging
import folium
import xml.etree.ElementTree as ET  # Import XML parsing library
import re

logging.basicConfig(level=logging.INFO)


def dijkstra(graph, start):
    try:
        times = {node: float('inf') for node in graph}
        times[start] = 0
        priority_queue = [(0, start)]

        while priority_queue:
            current_time, current_node = heapq.heappop(priority_queue)
            logging.debug("Current node: %s", current_node)

            if current_time > times[current_node]:
                continue
            for neighbor, edge_data in graph[current_node].items():
                if 'weight' in edge_data:
                    new_time = current_time + edge_data['weight']
                    if new_time < times[neighbor]:
                        times[neighbor] = new_time
                        heapq.heappush(priority_queue, (new_time, neighbor))
                else:
                    logging.warning("Edge data does not contain 'weight' attribute: %s", edge_data)
        logging.debug("Times: %s", times)
        return times
    except Exception as e:
        logging.error("Error occurred in Dijkstra's algorithm: %s", e)
        return None

def parse_gpx_data(gpx_data_string):
    waypoints = []
    try:
        import xml.etree.ElementTree as ET
        root = ET.fromstring(gpx_data_string)
        for trkpt in root.findall(".//{http://www.topografix.com/GPX/1/1}trkpt"):
            lat = float(trkpt.get("lat"))
            lon = float(trkpt.get("lon"))
            ele = float(trkpt.find("{http://www.topografix.com/GPX/1/1}ele").text)
            waypoints.append((lat, lon, ele))
    except Exception as e:
        print("Failed to parse GPX data from string.")
        print(e)
    return waypoints

def add_edge_weights(graph):
    for u, v, data in graph.edges(data=True):
        length = data.get('length', 1.0)
        road_type = data.get('highway', 'unknown')  # Assume 'highway' attribute contains road type
        road_condition = data.get('surface', 'unknown')  # Assume 'surface' attribute contains road condition
        # Assign weights based on road type and condition
        if road_type == 'track' and road_condition == 'gravel':
            weight = length * 0.8  # Adjust weight for gravel tracks
        elif road_type == 'path':
            weight = length * 1.2  # Adjust weight for paths
        else:
            weight = length  # Default weight
        data['weight'] = weight


def find_fastest_route(graph, start_node, end_node):
    try:
        times = dijkstra(graph, start_node)
        if times is None:
            raise ValueError("Failed to find the fastest route.")

        path = nx.shortest_path(graph, source=start_node, target=end_node, weight='weight')
        return path
    except Exception as e:
        logging.error("Error occurred in finding the fastest route: %s", e)
        return None


def convert_coordinates_to_nodes(graph, coordinates):
    nodes = []
    for coord in coordinates:
        node = ox.distance.nearest_nodes(graph, coord[0], coord[1])
        nodes.append(node)
    return nodes



def get_directions(graph, path):
    directions = []
    for i in range(len(path) - 1):
        start_node = path[i]
        end_node = path[i + 1]

        edge_data = graph[start_node][end_node]

        road_name = edge_data.get('name', 'Unnamed Road')
        road_type = edge_data.get('highway', 'Unknown')
        road_condition = edge_data.get('surface', 'Unknown')

        directions.append(f"Travel on {road_name} ({road_type}, {road_condition})")
    return directions


if __name__ == "__main__":
    # Get the directory of the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the file path to the GPX file
    gpx_file_path = os.path.join(script_dir, "current.gpx")

    # Read GPX data from file
    try:
        with open(gpx_file_path, "r") as f:
            gpx_data_from_file = f.read()
    except FileNotFoundError:
        print(f"Error: File '{gpx_file_path}' not found.")
        gpx_data_from_file = None

    # Parse the GPX data from the file
    waypoints_from_file = parse_gpx_data(gpx_data_from_file)

    # Example GPX data string
    your_gpx_data_string = """<your GPX data string here>"""

    # Parse the GPX data directly from string
    waypoints_from_string = parse_gpx_data(your_gpx_data_string)

    # Print waypoints from file
    print("Waypoints from file:")
    if waypoints_from_file:
        for i, coord in enumerate(waypoints_from_file, 1):
            print(f"Waypoint {i}: Latitude {coord[0]}, Longitude {coord[1]}, Elevation {coord[2]}")
    else:
        print("Failed to parse GPX data from file.")

    # Print waypoints from string
    print("\nWaypoints from string:")
    if waypoints_from_string:
        for i, coord in enumerate(waypoints_from_string, 1):
            print(f"Waypoint {i}: Latitude {coord[0]}, Longitude {coord[1]}, Elevation {coord[2]}")
    else:
        print("Failed to parse GPX data from string.")