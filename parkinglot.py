"""
# Map 4
# Road 11, 27, 47

START EGO:
Road Id: 26   Lane Id: 1
x = 284.636  y = -242.797  z = 0.067
EGO TARGET:
Road Id: 11  Lane Id: 1
x = 285.713  y = -172.552  z = 0.191

PLACE OTHER PARKING:
Road Id: 11   Ego Lane Id: 1
x = 290.908  y = -191.125  z = 0.225

PEDESTRIAN START:
Road Id: 468   Lane Id: -1
x = 303.319  y = -194.256  z = 0.166
PEDESTRIAN TARGET:
Road Id: 11    Lane Id: 1
x = 276.470  y = -190.128  z = 0.290
"""
import carla
import numpy as np

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
world.set_weather(carla.WeatherParameters().WetSunset) # currently, set manually
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
carla_map = world.get_map()

def spawn_vehicle(world, spawn_transform=None, vehicle_type=None, vehicle_color=None):
    """
    :param spawn_transform: if None, will randomize
    :param vehicle_type: if None, will randomize
    :param vehicle_color: if None, will randomize
    :return: carla vehicle object
    """
    if vehicle_type is None:
        vehicle_names = [v.id[8:] for v in world.get_blueprint_library().filter("vehicle")]
        vehicle_type = np.random.choice(vehicle_names)
    blueprint = world.get_blueprint_library().find("vehicle."+vehicle_type)
    if blueprint.has_attribute('color'):
        if vehicle_color is None:
            vehicle_color = f'{np.random.choice(256)},{np.random.choice(256)},{np.random.choice(256)}'
        blueprint.set_attribute('color', vehicle_color)
    if spawn_transform is None:
        spawn_transform = np.random.choice(world.get_map().get_spawn_points())
    spawn_transform.location.z += 0.1 # helps in case map starting locations are incorrect
    vehicle = world.spawn_actor(blueprint, spawn_transform)
    return vehicle

def spectator_camera_transform(actor):
    base_transform = actor.get_transform()
    heading = base_transform.rotation.get_forward_vector()
    x, y, z = base_transform.location.x, base_transform.location.y, base_transform.location.z
    right_vec = base_transform.rotation.get_right_vector()
    camera_location = carla.Location(x, y, z) - 8*heading #- 5*right_vec
    camera_location.z += 5
    pitch, yaw, roll = base_transform.rotation.pitch, base_transform.rotation.yaw, base_transform.rotation.roll
    camera_rotation = carla.Rotation(pitch-15, yaw, roll)
    return carla.Transform(camera_location, camera_rotation)

def main(actor_list):
    world.tick()
    ego = actor_list[0]
    ped = actor_list[1]
    while True:
        try:
            world.tick()
            accelerate(ego, 0.37)
            walk(ped, 0.87)
            world.get_spectator().set_transform(spectator_camera_transform(ego))
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            for actor in actor_list:
                carla.command.DestroyActor(actor)
            client.reload_world()
            break


def accelerate(vehicle, accel):
    control_command = carla.VehicleControl()
    control_command.throttle = accel
    control_command.steer = 0.0
    vehicle.apply_control(control_command)

def walk(pedestrian, speed):
    control_command = carla.WalkerControl()
    control_command.direction = carla.Vector3D(-1, 0, 0)
    control_command.speed = speed
    pedestrian.apply_control(control_command)

if __name__ == "__main__":
    # EGO
    ego_init = carla.Transform(carla.Location(285, -245, 0.1),
                               carla.Rotation(0, 90, 0))
    ego = spawn_vehicle(world, ego_init, "mercedes.coupe_2020")
    # Pedestrian
    ped_init = carla.Transform(carla.Location(302, -194, 0.2),
                               carla.Rotation(0,180,0))
    ped_blueprint = world.get_blueprint_library().find(f'walker.pedestrian.00{10+np.random.choice(38)}')
    pedestrian = world.spawn_actor(ped_blueprint, ped_init)
    world.tick()
    world.get_spectator().set_transform(spectator_camera_transform(ego))
    main([ego, pedestrian])