import carla
import numpy as np
import functools
import GeometricControl.pure_pursuit as pp

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)
carla_map = world.get_map()



WAYPOINT_DIST = 1
# This list of roads corresponds to the 8-shaped loop
TRACK_ROADS = [761, 36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
 41, 1400, 40, 1185, 39, 1092, 38, 1601, 37]
TRACK_ROADS = [38]
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

start_point =0
navigation_waypoints = waypoints[start_point:]
navigation_waypoints.reverse()
blueprint = world.get_blueprint_library().find('vehicle.audi.tt')
blueprint.set_attribute('color', '0,85,155')
vehicle_start = navigation_waypoints[0].transform
vehicle_start.location.z += 0.1 # The waypoint is *below* the road?! This avoids a collision
vehicle = world.spawn_actor(blueprint, vehicle_start)
world.get_spectator().set_transform(vehicle_start)

# breakpoint()
control = pp.PurePursuit(vehicle, navigation_waypoints)

# breakpoint()
while True:
    try:
        control.update()
        world.tick()
        continue
    except KeyboardInterrupt:
        print('\n Destroying all Actors')
        client.apply_batch([carla.command.DestroyActor(s) for s in static_items])
        carla.command.DestroyActor(vehicle)
        client.reload_world()
        break


def send_control_command(client, throttle, steer, brake,
                         hand_brake=False, reverse=False):
    """
    throttle in [0,1]
    steer in [-1,1]
    brake in [0,1]
    Automatic control of vehicle. (manual_gear_shift is False)
    """
    control = VehicleControl()
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control.steer = steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = hand_brake
    control.reverse = reverse
    client.send_control(control)