import os
import heapq
import logging
import xml.etree.ElementTree as ET
import osmnx as ox
import networkx as nx

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
        root = ET.fromstring(gpx_data_string)
        for trkpt in root.findall(".//{http://www.topografix.com/GPX/1/1}trkpt"):
            lat = float(trkpt.get("lat"))
            lon = float(trkpt.get("lon"))
            ele = float(trkpt.find("{http://www.topografix.com/GPX/1/1}ele").text)
            waypoints.append((lat, lon, ele))
    except Exception as e:
        logging.error("Failed to parse GPX data from string: %s", e)
    return waypoints


def add_edge_weights(graph):
    for u, v, data in graph.edges(data=True):
        length = data.get('length', 1.0)
        road_type = data.get('highway', 'unknown')
        road_condition = data.get('surface', 'unknown')

        if road_type == 'track' and road_condition == 'gravel':
            weight = length * 0.8
        elif road_type == 'path':
            weight = length * 1.2
        else:
            weight = length
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
        node = ox.distance.nearest_nodes(graph, coord[1], coord[0])
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

    graph = ox.graph_from_place("Piedmont, California, USA", network_type='bike')
    add_edge_weights(graph)

    gpx_data_string = """<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
<gpx version="1.1" creator="Brock" xmlns="http://www.topografix.com/GPX/1/1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">
  <metadata>
    <name>Route1</name>
    <desc>Route 1</desc>
    <author>
      <name>Brock</name>
    </author>
    <time>2024-05-16T12:00:00Z</time>
  </metadata>
  <wpt lat="37.7749" lon="-122.4194">
    <ele>10</ele>
    <time>2024-05-16T12:00:00Z</time>
    <name>Start</name>
    <desc>Starting point of the route</desc>
  </wpt>
  <wpt lat="37.7799" lon="-122.4144">
    <ele>15</ele>
    <time>2024-05-16T12:10:00Z</time>
    <name>Waypoint 1</name>
    <desc>First waypoint</desc>
  </wpt>
  <trk>
    <name>Route 2</name>
    <desc>Example track for mountain biking</desc>
    <trkseg>
      <trkpt lat="37.7749" lon="-122.4194">
        <ele>10</ele>
        <time>2024-05-16T12:00:00Z</time>
      </trkpt>
      <trkpt lat="37.7759" lon="-122.4184">
        <ele>12</ele>
        <time>2024-05-16T12:05:00Z</time>
      </trkpt>
      <trkpt lat="37.7769" lon="-122.4174">
        <ele>14</ele>
        <time>2024-05-16T12:10:00Z</time>
      </trkpt>
      <trkpt lat="37.7779" lon="-122.4164">
        <ele>16</ele>
        <time>2024-05-16T12:15:00Z</time>
      </trkpt>
      <trkpt lat="37.7789" lon="-122.4154">
        <ele>18</ele>
        <time>2024-05-16T12:20:00Z</time>
      </trkpt>
      <trkpt lat="37.7799" lon="-122.4144">
        <ele>20</ele>
        <time>2024-05-16T12:25:00Z</time>
      </trkpt>
    </trkseg>
  </trk>
</gpx>"""

    waypoints = parse_gpx_data(gpx_data_string)

    nodes = convert_coordinates_to_nodes(graph, waypoints)

    if len(nodes) >= 2:
        start_node = nodes[0]
        end_node = nodes[-1]

        path = find_fastest_route(graph, start_node, end_node)

        if path:
            directions = get_directions(graph, path)
            print("Turn-by-turn directions:")
            for direction in directions:
                print(direction)
        else:
            print("Failed to find a route.")
    else:
        print("Insufficient waypoints to determine a route.")
