"""
This module is a helper script for testing detection/classification NNs.
It first spawns a random NPC vehicle actor and takes a picture with an
RGB sensor (which is onboard an ego actor).
Then it displaces the ego actor (+ camera) and takes another picture.
"""

import carla
import numpy as np
import datetime, time
from queue import Queue
# np.random.seed(datetime.datetime.now().microsecond)

IMG_FILEPATH = 'images/'

# Weather settings:  # todo:
"""
 ['ClearNight',
'ClearNoon',
'ClearSunset',
'CloudyNight',
'CloudyNoon',
'CloudySunset',
'HardRainNight',
'HardRainNoon',
'HardRainSunset',
'MidRainSunset',
'MidRainyNight',
'MidRainyNoon',
'SoftRainNight',
'SoftRainNoon',
'SoftRainSunset',
'WetCloudyNight',
'WetCloudyNoon',
'WetCloudySunset',
'WetNight',
'WetNoon',
'WetSunset']
"""

# Create CARLA world
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
    camera_location = carla.Location(x, y, z) - 5*heading  + 5*right_vec
    camera_location.z += 3
    pitch, yaw, roll = base_transform.rotation.pitch, base_transform.rotation.yaw, base_transform.rotation.roll
    camera_rotation = carla.Rotation(pitch-15, yaw-30, roll)
    return carla.Transform(camera_location, camera_rotation)

def spawn_rgb_sensor(world, actor, resolution=1280, height_offset=0.2, pitch=-15, filepath=IMG_FILEPATH):
    """
    Camera is located *ON TOP* of vehicle (5/8 of the way to the front)
    (for cars with long hoods, this might be in an unrealistic place)
    :param actor: carla (ego) vehicle
    :param resolution: Returns square image of this size
    :return: camera actor
    """
    blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    blueprint.set_attribute('image_size_x', str(resolution))
    blueprint.set_attribute('image_size_y', str(resolution))
    height = 2 * actor.bounding_box.extent.z
    length = 2 * actor.bounding_box.extent.x
    camera_transform = carla.Transform(carla.Location(length/8, 0, height + height_offset),
                                       carla.Rotation(pitch,0,0))
    camera = world.spawn_actor(blueprint, camera_transform, attach_to=actor)
    camera.listen(lambda data: sensor_callback(data, filepath))
    return camera

def sensor_callback(sensor_data, filepath):
    global SAVE_IMG
    if SAVE_IMG:
        exp_time = datetime.datetime.now()
        exp_time_str = f'{exp_time.month}-{exp_time.day}_{exp_time.hour}{exp_time.minute}_{exp_time.second}-{exp_time.microsecond//1000}'
        sensor_data.save_to_disk(f'{filepath}test_{exp_time_str}')
        sensor_queue.put(sensor_data.frame)

def tick_N_times(world, N=1):
    for _ in range(N):
        world.tick()

def main():
    world.tick()
    while True:
        try:
            sensor_queue.get(True, 1.0) # Waits at most 1 second to give time to finish saving img
            world.tick()
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            carla.command.DestroyActor(ego)
            client.reload_world()
            break

def displaced_transform(actor, distance_ahead=20, lateral_offset=0, yaw_offset=0):
    """
    Given an actor, this transform is distance_ahead meters ahead.
    Possibility to offset laterally (+ is right, - is left)
    and to rotate the vehicle with yaw
    :param actor: vehicle object
    :return: vehicle object
    """
    base_transform = actor.get_transform()
    heading = base_transform.rotation.get_forward_vector()
    right_vec = base_transform.rotation.get_right_vector()
    new_location = base_transform.location + distance_ahead * heading + lateral_offset * right_vec
    pitch, yaw, roll = base_transform.rotation.pitch, base_transform.rotation.yaw, base_transform.rotation.roll
    new_rotation = carla.Rotation(pitch, yaw+yaw_offset, roll)
    return carla.Transform(new_location, new_rotation)

def print_actor_location(actor):
    location = actor.get_transform().location
    print(f'x = {location.x:.3f}  y = {location.y:.3f}  z = {location.z:.3f}')

if __name__ == "__main__":
    sensor_queue = Queue()
    ego = spawn_vehicle(world)
    world.tick()
    npc_transform = displaced_transform(ego)
    npc = spawn_vehicle(world, spawn_transform=npc_transform)
    world.get_spectator().set_transform(spectator_camera_transform(ego)) # Just for visualization
    world.tick()
    SAVE_IMG = False
    camera = spawn_rgb_sensor(world, ego)
    tick_N_times(world, N=15) # give time for scene to "settle"

    # Saves an image to disk
    SAVE_IMG = True
    world.tick()
    sensor_queue.get(True, 2.0) # waits at most 2 sec before continuing
    SAVE_IMG = False

    lat_offset = -1.5
    yaw_offset = -10
    new_npc_transform = displaced_transform(ego, lateral_offset=lat_offset, yaw_offset=yaw_offset)
    npc.set_transform(new_npc_transform)
    tick_N_times(world, N=8)

    # Saves an image to disk
    SAVE_IMG = True
    world.tick()
    sensor_queue.get(True, 2.0)
    SAVE_IMG = False

    carla.command.DestroyActor(ego)
    carla.command.DestroyActor(npc)
    carla.command.DestroyActor(camera)
    client.reload_world()
