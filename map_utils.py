"""
OpenDriveMap class
Contains methods to calculate "longitudinal" lane distance between two points of a map
"""
import xml.etree.ElementTree as ET
import networkx as nx
import collections
import numpy as np


class OpenDriveMap:
    def __init__(self, od_filepath, carla_map):
        self.od_filepath = od_filepath
        self.carla_map = carla_map

        self.tree = ET.parse(od_filepath)
        self.root = self.tree.getroot()
        self.map_elements = list(self.root) # includes 1 'header' multiple 'road' 'controller' 'junction'
        self.road_elements = self.root.findall('road')
        self.road_elements = [road for road in self.road_elements if self.is_drivable(road)]
        self.road_ids = [r.get('id') for r in self.road_elements]
        self.junction_elements = self.root.findall('junction')
        # print(f'{od_map} has {len(road_elements)} roads and {len(junction_elements)} junctions')
        self.nonjunc_roads = [road for road in self.road_elements if road.attrib['junction'] == '-1']
        self.waypoints = self.carla_map.generate_waypoints(0.3)  # waypoints spaced every X meters
        self.create_id_dicts()
        self.topology = self.minimum_topology()
        self.full_topology = self.road_network_topology()
        self.roadlane_topology = self.carla_roadlane_topology()
        self.dict_lane_diff = {}

    def create_id_dicts(self):
        """
        Creates a set of useful lookup tables where the key is the string id of the road/junction
        """
        # Contains All junctions
        self.id_dict_junction_elements = {j.get('id'): j for j in self.junction_elements}
        # Contains Roads which are not in a junction
        self.id_dict_nonjunc_roads = {r.get('id'): r for r in self.nonjunc_roads}
        # Contains all Roads
        self.id_dict_road_elements = {r.get('id'): r for r in self.road_elements}
        # Contains list of Roads which are inside a Junction
        self.id_dict_roads_injunc = {j.get('id'): \
                                           [c.get('connectingRoad') \
                                            for c in j.findall('connection') \
                                            if c.get('connectingRoad') in self.road_ids] \
                                     for j in self.junction_elements}
        # Contains list of waypoints inside junctions
        self.id_dict_junc_wp = collections.defaultdict(list)
        for waypoint in self.waypoints:
            if waypoint.is_junction:
                self.id_dict_junc_wp[f"{waypoint.road_id}"].append(waypoint)

    def shortest_path(self, road_id_1, road_id_2):
        return nx.shortest_path(self.topology, str(f"r{road_id_1}"), str(f"r{road_id_2}"))

    def is_drivable(self, road):
        """
        Returns True if road has at least one drivable lane
        False otherwise
        """
        sections = list(road.find('lanes').find('laneSection'))
        for section in sections:
            sect_lanes = section.findall('lane')
            for lane in sect_lanes:
                if lane.get('type') == 'driving':
                    return True
        return False

    def drivable_lanes(self, road):
        """
        Returns List of drivable lanes from a road
        """
        drivable_lanes = []
        sections = list(road.find('lanes').find('laneSection'))
        for section in sections:
            sect_lanes = section.findall('lane')
            for lane in sect_lanes:
                if lane.get('type') == 'driving':
                    drivable_lanes.append(int(lane.get('id')))
        return drivable_lanes

    def road_network_topology(self):
        """
        excludes junctions as separate elements
        only roads are in graph
        """
        graph = nx.DiGraph()
        for road in self.road_elements:
            road_id = road.get('id')
            links = road.find('link')
            for link in links:
                if link.get('elementType')=='road':
                    link_id = link.get('elementId')
                    graph.add_edge(f"{road_id}", f"{link_id}", distance=float(road.get('length')))
                    link_road = self.road_from_id(link_id)
                    graph.add_edge(f"{link_id}", f"{road_id}", distance=float(link_road.get('length')))
        return graph

    def minimum_topology(self):
        """
        uses only list of nonjunc_roads
        junctions are implied from the list of road successor/predecessor
        return: nx graph where nodes are roads or junctions

        assumption: The map has no Junction-Junction connections.
        I.e. when driving in a junction, all possible next roads are non-junc roads
        """
        graph_topology = nx.Graph()
        for road in self.nonjunc_roads:
            road_id = road.get('id')
            links = road.find('link')
            for link in links:
                l_type = link.get('elementType')[0]
                l_id = link.get('elementId')
                if link.tag == 'predecessor':  # only predecessor or successor are defined
                    graph_topology.add_edge(f'{l_type[0]}{l_id}', f'r{road_id}')
                else:
                    graph_topology.add_edge(f'r{road_id}', f'{l_type[0]}{l_id}')
        return graph_topology

    def carla_roadlane_topology(self):
        """
        # Do Not Use
        Creates Minimum Topology from Carla Map
        Returns nx.Graph with the topology
        The topology nodes are (road,lane) so hard to use.
        """
        def wp_data(waypoint):
            return (waypoint.road_id, waypoint.lane_id)
        raw_topology = self.carla_map.get_topology()
        topology = []
        for edge in raw_topology:
            r0, l0 = wp_data(edge[0])
            r1, l1 = wp_data(edge[1])
            e = ((r0, l0), (r1, l1))
            topology.append(e)
            # topology.append((edge[0].road_id, edge[1].road_id))
        graph_topology = nx.Graph()
        graph_topology.add_edges_from(topology)
        return graph_topology

    def find_lane_direction(self, road, lane_id):
        for side in list(road.find('lanes').find('laneSection')):
            for lane in list(side):
                if lane.get('id') == lane_id:
                    return lane.find('userData').find('vectorLane').get('travelDir')

    def _lane_dist_between_wp(self, waypoint1, waypoint2):
        if waypoint1.is_junction and waypoint2.is_junction:
            if waypoint1.junction_id==waypoint2.junction_id:
                return 0
        road1, lane1 = waypoint1.road_id, waypoint1.lane_id
        road2, lane2 = waypoint2.road_id, waypoint2.lane_id
        road_1_drivable_lanes = self.drivable_lanes(self.id_dict_road_elements[str(road1)])
        if ((road1, lane1), (road2, lane2)) in self.dict_lane_diff:
            return self.dict_lane_diff[((road1, lane1), (road2, lane2))]
        elif ((road2, lane2), (road1, lane1)) in self.dict_lane_diff:
            return self.dict_lane_diff[((road2, lane2), (road1, lane1))]
        else:
            path_dist = [nx.shortest_path_length(self.roadlane_topology, source=(road1, ll),
                                          target=(road2,lane2))
                         for ll in road_1_drivable_lanes]
            if min(path_dist) == path_dist[road_1_drivable_lanes.index(lane1)]:
                lane_dist = 0
            else:
                min_lane = road_1_drivable_lanes[np.argmin(path_dist)]
                lane_dist = min_lane - lane1
                if np.sign(min_lane) != np.sign(lane1):
                    lane_dist += np.sign(lane1)
                #todo: how does traffic direction affect this?
                if self.find_lane_direction(self.id_dict_road_elements[str(road1)], str(lane1)) == 'backward':
                    lane_dist *= -1
            self.dict_lane_diff[((road1, lane1), (road2, lane2))] = lane_dist
            self.dict_lane_diff[((road2, lane2), (road1, lane1))] = lane_dist
            return lane_dist


    def _longitudinal_dist_between_wp(self, waypoint1, waypoint2):
        """
        Assumes roads on road_list are actually connected! It will only sum the
        lengths of the roads and not check connectivity
        """
        if waypoint1.is_junction and waypoint2.is_junction:
            if waypoint1.junction_id==waypoint2.junction_id:
                return self.waypoint_distance(waypoint1, waypoint2)
        road_id1, lane1, s1 = str(waypoint1.road_id), waypoint1.lane_id, waypoint1.s
        road_id2, lane2, s2 = str(waypoint2.road_id), waypoint2.lane_id, waypoint2.s
        shortest_path = nx.shortest_path_length(self.full_topology, source=road_id1,
                                                target=road_id2, weight="distance")
        if shortest_path == 0:  # checks to see if both wp in same road
            return abs(s1 - s2)
        road1 = self.road_from_id(road_id1)
        road2 = self.road_from_id(road_id2)
        shortest_path -= float(road1.get('length'))
        if self.find_lane_direction(road1, str(lane1)) == 'forward':
            shortest_path -= s1
            shortest_path += float(road1.get('length'))
        elif self.find_lane_direction(road1, str(lane1)) == 'backward':
            shortest_path += s1
        if self.find_lane_direction(road2, str(lane2)) == 'forward':
            shortest_path += s2
        elif self.find_lane_direction(road2, str(lane2)) == 'backward':
            shortest_path += float(road2.get('length'))
            shortest_path -= s2
        return shortest_path

    def _x_old_long_dist(self):
        """
        road1_dist = s1
        road2_dist = s2
        if self.find_lane_direction(road1, str(lane1)) == 'forward':
            road1_dist *= -1
            road1_dist += float(road_list[0].get('length'))
        if self.find_lane_direction(road_list[-1], str(lane2)) == 'backward':
            road2_dist *= -1
            road2_dist += float(road_list[-1].get('length'))
        intermediate_dist = 0
        for road in road_list[1:-1]:
            intermediate_dist += float(road.get('length'))
        return road1_dist + road2_dist + intermediate_dist
        """
        pass

    def road_from_id(self, road_id):
        """
        Returns opendrive road objects that has that id
        """
        ii = self.road_ids.index(str(road_id))
        return self.road_elements[ii]

    def road_and_lane_graph_shortest_path(self, roadlane1, roadlane2):
        """
        roadlane = (vehicle_wp.road_id, vehicle_wp.lane_id)   tuples
        """
        return nx.shortest_path(self.carla_topology, roadlane1, roadlane2)

    def feasible_waypoints(self, waypoint, tolerance=0.7):
        """
        Given a waypoint that is in a junction,
        returns list of other waypoints which might also be feasible.
        I.e. the car might be on the other road, due to overlapping definitions
        """
        # if the waypoint is on a nonjunction road
        if not waypoint.is_junction:
            return [waypoint]
        else:
            feasible_waypoints = []
            juncroad_ids = self.id_dict_roads_injunc[str(waypoint.junction_id)]
            for road_id in juncroad_ids:
                distances = [self.waypoint_distance(waypoint, wp) for wp in self.id_dict_junc_wp[road_id]]
                dd = np.argmin(distances)
                if distances[dd] <= tolerance:
                    feasible_waypoints.append(self.id_dict_junc_wp[road_id][dd])
            assert len(feasible_waypoints)>0
            return feasible_waypoints

    def waypoint_coordinates(self, waypoint):
        return np.array([waypoint.transform.location.x,
                         waypoint.transform.location.y,
                         waypoint.transform.location.z])

    def waypoint_orientation(self, waypoint):
        """ unit vector (object) pointing in the road direction"""
        return waypoint.transform.rotation.get_forward_vector()

    def location_orientation(self, location):
        waypoint = self.carla_map.get_waypoint(location)
        return self.waypoint_orientation(waypoint)

    def waypoint_distance(self, waypoint1, waypoint2):
        return np.linalg.norm(self.waypoint_coordinates(waypoint1) - self.waypoint_coordinates(waypoint2))

    def longitudinal_road_distance(self, v1_loc, v2_loc, verbose=False):
        """
        v_location = vehicle.get_location() object from Carla
        Project to waypoint - check feasible waypoints (if in junction)
        """
        wp1, wp2 = self.carla_map.get_waypoint(v1_loc,project_to_road=False), self.carla_map.get_waypoint(v2_loc,project_to_road=False)
        feasible_wp1 = self.feasible_waypoints(wp1)
        feasible_wp2 = self.feasible_waypoints(wp2)
        distance = 10*self.waypoint_distance(wp1, wp2)
        for f1 in feasible_wp1:
            for f2 in feasible_wp2:
                route_distance = self._longitudinal_dist_between_wp(f1, f2)
                if verbose:
                    print(route_distance)
                if route_distance < distance:
                    distance = route_distance
        return distance

    def lateral_road_distance(self, v1_loc, v2_loc):
        wp1, wp2 = self.carla_map.get_waypoint(v1_loc, project_to_road=False), self.carla_map.get_waypoint(v2_loc, project_to_road=False)
        feasible_wp1 = self.feasible_waypoints(wp1)
        feasible_wp2 = self.feasible_waypoints(wp2)
        distance = 4
        for f1 in feasible_wp1:
            for f2 in feasible_wp2:
                lane_distance = self._lane_dist_between_wp(f1, f2)
                if abs(lane_distance) < abs(distance):
                    distance = lane_distance
        return abs(distance*wp1.lane_width)

    def dist_to_end_of_road(self, v_wp, junction_crossed):
        if v_wp.is_junction:
            return 0
        road = self.road_from_id(v_wp.road_id)
        direction = self.find_lane_direction(road, str(v_wp.lane_id))
        road_length = float(road.get('length'))
        if direction == "forward":
            if not junction_crossed:
                return road_length - v_wp.s
            else:
                return v_wp.s
        elif direction == "backward":
            if not junction_crossed:
                return v_wp.s
            else:
                return road_length - v_wp.s