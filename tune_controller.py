import carla
import numpy as np
import GeometricControl.pure_pursuit as pp

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
world.set_weather(carla.WeatherParameters().ClearNight)
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

def evaluate(controller):
    travelled = (len(navigation_waypoints) - len(controller.waypoints))/len(navigation_waypoints)
    error = controller.cumulative_error
    return np.array([travelled, error])

def random_color():
    return f"{int(255*np.random.rand())},{int(255*np.random.rand())},{int(255*np.random.rand())}"

def tune(num_vehicles=1, spawn_every=80, max_steps=1200, **kwargs):
    world.tick()
    vehicles, controllers = [], []
    controller_step = []
    evaluation = []
    for t in range(max_steps+num_vehicles*spawn_every):
        try:
            for i, c in enumerate(controllers):
                if controller_step[i]<max_steps:
                    c.update()
                controller_step[i] += 1
            if t%spawn_every==1 and len(vehicles)<num_vehicles:
                try:
                    ego = spawn_vehicle(ego_spawn_transform, vehicle_color=random_color())
                    vehicles.append(ego)
                    controllers.append(pp.PurePursuit(ego, navigation_waypoints, world, **kwargs))
                    controller_step.append(0)
                except:
                    pass
            world.tick()
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            client.apply_batch([carla.command.DestroyActor(v) for v in vehicles])
            client.reload_world()
            break
    for c in controllers:
        evaluation.append(evaluate(c))
    evaluation = np.array(evaluation).T
    mean_dist = np.mean(evaluation[0])
    mean_abs_error = np.mean(evaluation[1])
    client.apply_batch([carla.command.DestroyActor(v) for v in vehicles])
    return mean_dist, mean_abs_error

def run_world():
    while True:
        try:
            world.tick()
            continue
        except KeyboardInterrupt:
            client.reload_world()
            break



if __name__ == "__main__":
    WAYPOINT_DIST = 1
    # This list of roads corresponds to the 8-shaped loop
    TRACK_ROADS = [1092, 38, 1601, 37, 761, 36, 862, 35, 43, 266, 42, 50, 1174, 49, 902, 48, 775, 47, 1073, 46, 144, 45, 6,
                       41, 1400, 40]  #, 1185, 39]
    waypoints = [w for w in carla_map.generate_waypoints(WAYPOINT_DIST)
                 if w.road_id==TRACK_ROADS[0] and w.lane_id == 4]
    waypoints.reverse()
    navigation_waypoints = create_location_waypoints(TRACK_ROADS)
    ego_spawn_transform = waypoints[0].transform
    # ego = spawn_vehicle(ego_spawn_transform)
    # world.get_spectator().set_transform(ego_spawn_transform)
    # control = pp.PurePursuit(ego, navigation_waypoints, world)
    vcd = [(vel, cons, deltmu) for vel in np.linspace(2, 6, 4) for cons in np.linspace(0, 10, 3) for deltmu in np.linspace(2, 3, 4)]
    results = {}
    for vel, cons, deltmu in vcd:
        params = {'kv': vel, "kc": cons, "delta_mul" : deltmu, "target_speed_mps": 85}
        performance = tune(num_vehicles=15, max_steps=1800, **params)
        results[(vel,cons,deltmu)] = performance
        print(f"kv= {vel} kc= {cons} deltmu= {deltmu} travelled= {performance[0]:.2f}  abs_error= {performance[1]:.2f}\n")
    print(results)
    run_world()

"""
kv= 2.0 kc= 0.0 deltmu= 2.0 travelled= 0.93  abs_error= 1746.80  65kph
"""