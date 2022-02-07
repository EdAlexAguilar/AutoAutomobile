import carla
import map_utils


od_map = 'OpenDriveMaps/Town05.xodr'
print(f'Using Map: {od_map[:-5]}')

client = carla.Client('localhost', 2000)
world = client.load_world(od_map[-11:-5])
world.set_weather(carla.WeatherParameters().ClearSunset)
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
carla_map = world.get_map()

traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_global_distance_to_leading_vehicle(1.0)
traffic_manager.set_synchronous_mode(True)

proc_map = map_utils.OpenDriveMap(od_map, carla_map)


world.tick()
"""
while True:
    try:
        world.tick()
        continue
    except KeyboardInterrupt:
        # print('\n Destroying all Actors')
        # carla.command.DestroyActor(ego)
        client.reload_world()
        break
"""