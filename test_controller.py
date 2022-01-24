import carla
import numpy as np
import GeometricControl.pure_pursuit as pp

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
world.set_weather(carla.WeatherParameters().CloudySunset)
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
    vehicle_start = spawn_transform
    vehicle_start.location.z += 0.1 # The waypoint is *below* the road?! This avoids a collision
    if random_start:
        vehicle_start.location.x += 3*(np.random.rand()-0.5)
        vehicle_start.location.y += 3*(np.random.rand()-0.5)
        original_yaw = vehicle_start.rotation.yaw
        vehicle_start_rotation = carla.Rotation(yaw=original_yaw + 80*(np.random.rand()-0.5))
        vehicle_start.rotation = vehicle_start_rotation
    vehicle = world.spawn_actor(blueprint, vehicle_start)
    return vehicle

def main():
    world.tick()
    vehicles, controllers = [], []
    num_vehicles = 5
    spawn_every = 60
    t = 0
    while True:
        try:
            t += 1
            for c in controllers:
                c.update()
            if t%spawn_every==1 & len(vehicles)<num_vehicles:
                ego = spawn_vehicle(ego_spawn_transform)
                vehicles.append(ego)
                controllers.append(pp.PurePursuit(ego, navigation_waypoints, world))
            world.tick()
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            client.apply_batch([carla.command.DestroyActor(v) for v in vehicles])
            # carla.command.DestroyActor(ego)
            client.reload_world()
            break

if __name__ == "__main__":
    WAYPOINT_DIST = 1
    # This list of roads corresponds to the 8-shaped loop
    TRACK_ROADS = [36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
                       41, 1400, 40]  # , 1185, 39, 1092, 38, 1601, 37, 761]
    waypoints = [w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
                 if w.road_id==TRACK_ROADS[0] and w.lane_id == 4]
    navigation_waypoints = create_location_waypoints(TRACK_ROADS)
    ego_spawn_transform = waypoints[0].transform
    # ego = spawn_vehicle(ego_spawn_transform)
    world.get_spectator().set_transform(ego_spawn_transform)
    # control = pp.PurePursuit(ego, navigation_waypoints, world)
    main()

