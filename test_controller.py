import carla
import numpy as np
import GeometricControl.pure_pursuit as pp

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
world.set_weather(carla.WeatherParameters().ClearSunset)
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
carla_map = world.get_map()

def create_location_waypoints(road_list, lane=4):
    """
    :param road_list: road_id (int) list
    """
    navigation_waypoints = [w.transform.location for w in carla_map.generate_waypoints(WAYPOINT_DIST)
                            if w.road_id==road_list[0] and w.lane_id==lane]
    navigation_waypoints.reverse()
    for road_num in road_list[1:]:
        road_wp = [w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
                if w.road_id==road_num and w.lane_id==lane]
        road_wp = [w.transform.location for w in road_wp]
        last_nav_point = navigation_waypoints[-1]
        if last_nav_point.distance(road_wp[-1]) < last_nav_point.distance(road_wp[0]):
            road_wp.reverse()
        navigation_waypoints = navigation_waypoints + road_wp
    return navigation_waypoints

def spawn_vehicle(spawn_transform, random_start=True, vehicle_type="audi.tt", vehicle_color="0,65,120"):
    blueprint = world.get_blueprint_library().find("vehicle."+vehicle_type)
    blueprint.set_attribute('color', vehicle_color)
    x, y, z = spawn_transform.location.x, spawn_transform.location.y, spawn_transform.location.z
    veh_loc = carla.Location(x,y,z+0.1)
    veh_rot = carla.Rotation(0, spawn_transform.rotation.yaw, 0)
    if random_start:
        veh_loc.x += 3*(np.random.rand()-0.5)
        veh_loc.y += 3*(np.random.rand()-0.5)
        veh_rot.yaw += 80*(np.random.rand()-0.5)
    vehicle_start = carla.Transform(veh_loc, veh_rot)
    vehicle = world.spawn_actor(blueprint, vehicle_start)
    return vehicle

def camera_transform(actor):
    base_transform = actor.get_transform()
    heading = base_transform.rotation.get_forward_vector()
    x, y, z = base_transform.location.x, base_transform.location.y, base_transform.location.z
    right_vec = base_transform.rotation.get_right_vector()
    camera_location = carla.Location(x, y, z) - 15*heading # + 8*right_vec
    camera_location.z += 25
    pitch, yaw, roll = base_transform.rotation.pitch, base_transform.rotation.yaw, base_transform.rotation.roll
    camera_rotation = carla.Rotation(pitch-35, yaw, roll)
    return carla.Transform(camera_location, camera_rotation)


def main():
    world.tick()
    while True:
        try:
            control.update()
            world.get_spectator().set_transform(camera_transform(ego))
            world.tick()
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            carla.command.DestroyActor(ego)
            client.reload_world()
            break

if __name__ == "__main__":
    WAYPOINT_DIST = 1
    # This list of roads corresponds to the 8-shaped loop
    TRACK_ROADS = [1092, 38, 1601, 37, 761, 36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
                       41, 1400, 40]  # , 1185, 39, 1092, 38, 1601, 37, 761]
    waypoints = [w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
                 if w.road_id==TRACK_ROADS[0] and w.lane_id == 4]
    waypoints.reverse()
    navigation_waypoints = create_location_waypoints(TRACK_ROADS)
    ego_spawn_transform = waypoints[0].transform
    ego = spawn_vehicle(ego_spawn_transform)
    # ego.show_debug_telemetry()
    world.get_spectator().set_transform(ego_spawn_transform)
    params = {'kv': 2, "kc": 0, "delta_mul": 2, "target_speed_mps": 10}
    control = pp.PurePursuit(ego, navigation_waypoints, world, **params)
    main()

