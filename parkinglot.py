"""
# Map 4
# Road 11, 27, 47

START EGO:
Road Id: 26   Lane Id: 1
x = 284.636  y = -242.797  z = 0.067
EGO TARGET:
Road Id: 11  Lane Id: 1
x = 285.713  y = -172.552  z = 0.191

PEDESTRIAN START:
Road Id: 468   Lane Id: -1
x = 303.319  y = -194.256  z = 0.166
PEDESTRIAN TARGET:
Road Id: 11    Lane Id: 1
x = 276.470  y = -190.128  z = 0.290
"""
import carla
import numpy as np
from utils.carla_utils import spawn_vehicle, spectator_camera_transform, spawn_rgb_sensor
from utils.AEB_cruise import AEBCruise

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
carla_map = world.get_map()


# LOCATION OF OTHER PARKED VEHICLES, JUST FOR VISUALIZATION PURPOSES
other_parked = [(carla.Transform(carla.Location(280, -205, 0.25), carla.Rotation(0, 188, 0)),'volkswagen.t2_2021'),
                (carla.Transform(carla.Location(290.2, -201.5, 0.22)),'nissan.patrol'),
                (carla.Transform(carla.Location(289.7, -198, 0.22), carla.Rotation(0, 185, 0)),'seat.leon'),
                (carla.Transform(carla.Location(290.2, -191, 0.22), carla.Rotation(0, -10, 0)),'audi.etron'),
                (carla.Transform(carla.Location(298, -201, 0.2), carla.Rotation(0, 177, 0)),'jeep.wrangler_rubicon'),
                (carla.Transform(carla.Location(299, -211, 0.2), carla.Rotation(0, 185, 0)),'mini.cooper_s'),
                (carla.Transform(carla.Location(280.9, -195, 0.28), carla.Rotation(0, 8, 0)),'toyota.prius')]

# WEATHER AND TIME OF DAY OPTIONS
common_weather = {'sun_azimuth_angle': -110}
night = {'sun_altitude_angle': -3}  # dusk
day = {'sun_altitude_angle': 10}  # afternoon, sun in horizon
clear = {}
fog = {'precipitation_deposits': 70,
       'fog_density': 75,
       'cloudiness': 100}
time_of_day = {'day': day, 'night': night}
clarity = {'clear': clear, 'fog': fog}
# PEDESTRIAN BLUEPRINTS
pedestrian_blueprints = {'child': world.get_blueprint_library().find(f'walker.pedestrian.0010'),
                         'adult': world.get_blueprint_library().find(f'walker.pedestrian.0028')}



def main(actor_list):
    world.tick()
    ego = actor_list[0]
    ped = actor_list[1]
    # ego controller
    ego_cruise_control = AEBCruise(ego, target_speed=7, target_yaw=90)
    while True:
        try:
            world.tick()
            # accelerate(ego, 0.37)
            ego_cruise_control.control()
            walk(ped, 1.4)  # from wikipedia https://en.wikipedia.org/wiki/Preferred_walking_speedes
            world.get_spectator().set_transform(spectator_camera_transform(ego))
            # print(f'Ego Speed: {ego.get_velocity().length():.2f} m/s    Yaw: {ego.get_transform().rotation.yaw:.2f} ')
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            for actor in actor_list:
                carla.command.DestroyActor(actor)
            client.reload_world()
            break


def walk(pedestrian, speed):
    control_command = carla.WalkerControl()
    control_command.direction = carla.Vector3D(-1, 0, 0)
    control_command.speed = speed
    pedestrian.apply_control(control_command)


#
""" SCENARIO VARIATION OPTIONS """
#
TIME_OF_DAY = 'day'  # options: "day", "night"
CLARITY = 'clear'   # options: "clear", "fog"
PEDESTRIAN = 'adult'  # options: "child", "adult"


if __name__ == "__main__":
    weather_params = {**common_weather, **time_of_day[TIME_OF_DAY], **clarity[CLARITY]}
    world.set_weather(carla.WeatherParameters(**weather_params))
    actors = []
    # EGO
    ego_init = carla.Transform(carla.Location(286, -245, 0.1),
                               carla.Rotation(0, 90, 0))
    ego = spawn_vehicle(world, ego_init, "mercedes.coupe_2020")
    actors.append(ego)
    # Pedestrian
    #ped_init = carla.Transform(carla.Location(302, -194, 0.2),
    #                           carla.Rotation(0,180,0))
    ped_init = carla.Transform(carla.Location(297, -194, 0.25),
                               carla.Rotation(0,180,0))
    pedestrian = world.spawn_actor(pedestrian_blueprints[PEDESTRIAN], ped_init)
    actors.append(pedestrian)
    # Parked cars
    for park_init, park_model in other_parked:
        parked_npc = spawn_vehicle(world, park_init, park_model)
        actors.append(parked_npc)
    world.tick()
    # breakpoint()
    world.get_spectator().set_transform(spectator_camera_transform(ego))
    main(actors)  # actor[0] = ego ; actor[1] = pedestrian



# LEGACY JUNK
"""
def accelerate(vehicle, accel):
    control_command = carla.VehicleControl()
    control_command.throttle = accel
    control_command.steer = 0.0
    vehicle.apply_control(control_command)
"""