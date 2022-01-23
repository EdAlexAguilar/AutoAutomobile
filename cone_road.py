import carla
import numpy as np
import functools
import GeometricControl.pure_pursuit as pp

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

carla_map = world.get_map()

WAYPOINT_DIST = 1
# This list of roads corresponds to the 8-shaped loop
ALL_TRACK_ROADS = [36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
 41] #, 1400, 40, 1185, 39, 1092, 38, 1601, 37, 761]
TRACK_ROADS = [36]
waypoints =[w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
            if w.road_id in TRACK_ROADS and w.lane_id==4]

"""
import map_utils
od_map = 'OpenDriveMaps/Town04.xodr'
processed_map = map_utils.OpenDriveMap(od_map, carla_map)
TRACK_LENGTH = 0
for track_road in TRACK_ROADS:
    od_road = processed_map.road_from_id(str(track_road))
    TRACK_LENGTH += float(od_road.get('length'))
# print(TRACK_LENGTH) # =  3049.9244
"""

"""
CONES_EVERY = 15
static_waypoints =[]
static_waypoints.append([w for w in carla_map.generate_waypoints(CONES_EVERY) if w.road_id in TRACK_ROADS and w.lane_id==5])
static_waypoints.append([w for w in carla_map.generate_waypoints(CONES_EVERY) if w.road_id in TRACK_ROADS and w.lane_id==3])
static_items = []
print(f"Spawning {functools.reduce(lambda count, l: count + len(l), static_waypoints, 0)} Static Items.")
for sublist in static_waypoints:
    for wp in sublist:
        blueprint = world.get_blueprint_library().find(f'static.prop.trafficcone01')
        static_object = world.spawn_actor(blueprint, wp.transform)
        static_items.append(static_object)
"""
static_items = []


navigation_waypoints = [w.transform.location for w in waypoints]

for road_num in ALL_TRACK_ROADS[1:]:
    road_wp = [w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
            if w.road_id==road_num and w.lane_id==4]
    road_wp = [w.transform.location for w in road_wp]
    last_nav_point = navigation_waypoints[-1]
    if last_nav_point.distance(road_wp[-1]) < last_nav_point.distance(road_wp[0]):
        road_wp.reverse()
    navigation_waypoints = navigation_waypoints + road_wp


blueprint = world.get_blueprint_library().find('vehicle.audi.tt')
blueprint.set_attribute('color', '0,85,155')
vehicle_start = waypoints[0].transform
vehicle_start.location.z += 0.1 # The waypoint is *below* the road?! This avoids a collision
vehicle_start.location.x += 3*(np.random.rand()-0.5)
vehicle_start.location.y += 3*(np.random.rand()-0.5)
original_yaw = vehicle_start.rotation.yaw
vehicle_start_rotation = carla.Rotation(yaw=original_yaw + 80*(np.random.rand()-0.5))
vehicle_start.rotation = vehicle_start_rotation
vehicle = world.spawn_actor(blueprint, vehicle_start)

world.get_spectator().set_transform(vehicle_start)

control = pp.PurePursuit(vehicle, navigation_waypoints, world)

transform = vehicle.get_transform()
location = transform.location

world.tick()
# breakpoint()

i = 1

world.tick()
while True:
    try:
        control.update()
        i += 1
        if i%20 == 0:
            # speed = vehicle.get_velocity()
            # speed = speed.length()
            # print(f"speed = {speed*3.6:.2f} kph")
            # transform = vehicle.get_transform()
            # location = transform.location
            # print(f"LOCATION: x = {location.x}  y = {location.y}  z = {location.z}")
            # heading = transform.rotation.get_forward_vector()
            # print(f"HEADING: x = {heading.x}  y = {heading.y}  z = {heading.z}")
            # print(f"HEADING: yaw = {transform.rotation.yaw} \n")
            # dists = [location.distance(w) for w in navigation_waypoints]
            # print(f"{dists} \n\n ")
        world.tick()
        continue
    except KeyboardInterrupt:
        print('\n Destroying all Actors')
        client.apply_batch([carla.command.DestroyActor(s) for s in static_items])
        carla.command.DestroyActor(vehicle)
        client.reload_world()
        break


