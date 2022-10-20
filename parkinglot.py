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

"""
import carla
import numpy as np
from utils.carla_utils import spawn_vehicle, spectator_camera_transform, spawn_rgb_sensor, spawn_collision_sensor
import utils.carla_utils as carla_utils
from utils.AEB_cruise import AEBCruise
import time
from yolov7.main_detector import YOLODetector
import csv
import itertools

client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.04
world.apply_settings(settings)
# carla_map = world.get_map()
# EPISODE ENDING CONDITIONS
EPISODE_LENGTH = 20
PED_LIMIT_X = 282.5 # lower
EGO_LIMIT_Y = -185 # upper
# COORDINATE OFFSETS
X_OFFSET = 286
Y_OFFSET = -193.5

# LOCATION OF OTHER PARKED VEHICLES, JUST FOR VISUALIZATION PURPOSES
other_parked = [(carla.Transform(carla.Location(280, -205, 0.25), carla.Rotation(0, 188, 0)),'volkswagen.t2_2021'),
                (carla.Transform(carla.Location(290.2, -201.5, 0.22)),'nissan.patrol'),
                (carla.Transform(carla.Location(289.7, -198, 0.22), carla.Rotation(0, 185, 0)),'seat.leon'),
                (carla.Transform(carla.Location(290.2, -191, 0.22), carla.Rotation(0, -10, 0)),'audi.etron'),
                (carla.Transform(carla.Location(298, -201, 0.2), carla.Rotation(0, 177, 0)),'jeep.wrangler_rubicon'),
                (carla.Transform(carla.Location(299, -211, 0.2), carla.Rotation(0, 185, 0)),'mini.cooper_s'),
                (carla.Transform(carla.Location(280.9, -195, 0.28), carla.Rotation(0, -5, 0)),'toyota.prius')]

# WEATHER AND TIME OF DAY OPTIONS
common_weather = {'sun_azimuth_angle': -110}
night = {'sun_altitude_angle': -3}  # dusk
day = {'sun_altitude_angle': 10}  # afternoon, sun in horizon
clear = {}
fog = {'precipitation_deposits': 70,
       'fog_density': 85,
       'cloudiness': 100}
time_of_day = {'day': day, 'night': night}
clarity = {'clear': clear, 'fog': fog}
# PEDESTRIAN BLUEPRINTS
pedestrian_blueprints = {'child': world.get_blueprint_library().find(f'walker.pedestrian.0010'),
                         'adult': world.get_blueprint_library().find(f'walker.pedestrian.0028')}

# SENSOR & YOLO  (Initial loading is slow - so only do it once)
RESOLUTION = 1280
perception_model = YOLODetector(img_size=RESOLUTION, conf_threshold=0.87)
# perception_model = None

def yolo_predict(image):
    prediction = perception_model.detect(image)
    pred = prediction[0].detach().cpu().numpy()
    if pred.shape[0] == 0:
        pred = np.zeros((1, 6))
        pred[0, -1] = -1  # to make sure that class is not accidently considered '0'
    # Check if a pedestrian was observed or not
    person_probs = np.array([pred[i][-2] if pred[i][-1]==0 else -1 for i in range(pred.shape[0])])
    if 0 in pred[:,-1]:
        return True, max(person_probs)
    else:
        return False, max(person_probs)

"""
def main(actor_list):
    world.tick()
    initial_time = world.get_snapshot().elapsed_seconds
    simulator_time = initial_time
    ego = actor_list[0]
    ped = actor_list[1]
    # ego controller
    ego_cruise_control = AEBCruise(ego, target_speed=8, target_yaw=90)
    carla_utils.frame_counter = -1
    current_frame = 0
    while (simulator_time - initial_time)<EPISODE_LENGTH:
        try:
            world.tick()
            # COLLECT DATA
            simulator_time = world.get_snapshot().elapsed_seconds
            time_trace.append(round(simulator_time-initial_time, 3))
            update_actor_data(ego, ego_data, x_offset=X_OFFSET, y_offset=Y_OFFSET)
            update_actor_data(ped, ped_data, x_offset=X_OFFSET, y_offset=Y_OFFSET)
            # PROCESS FOR NEXT FRAME
            while current_frame != carla_utils.frame_counter:
                time.sleep(0.01)
            current_frame += 1
            cam_img = carla_utils.array_output
            ped_in_sight, prob = yolo_predict(cam_img)
            print(f'Pedestrian In Sight: {ped_in_sight}   Prob: {prob}')
            ego_cruise_control.control(emergency_brake=ped_in_sight)
            walk(ped, 1.4)  # from wikipedia https://en.wikipedia.org/wiki/Preferred_walking_speed
            world.get_spectator().set_transform(spectator_camera_transform(ego))
            # print(f'Ego Speed: {ego.get_velocity().length():.2f} m/s    Yaw: {ego.get_transform().rotation.yaw:.2f} ')
            continue
        except KeyboardInterrupt:
            print('\n Destroying all Actors')
            for actor in actor_list:
                carla.command.DestroyActor(actor)
            client.reload_world()
            break
"""

def update_actor_data(actor, actor_data, prefix='', x_offset=0, y_offset=0):
    actor_data[prefix+'x'].append(actor.get_location().x - x_offset)
    actor_data[prefix+'y'].append(actor.get_location().y - y_offset)
    actor_data[prefix+'v_x'].append(actor.get_velocity().x)
    actor_data[prefix+'v_y'].append(actor.get_velocity().y)
    actor_data[prefix+'a_x'].append(actor.get_acceleration().x)
    actor_data[prefix+'a_y'].append(actor.get_acceleration().y)

def walk(pedestrian, speed):
    control_command = carla.WalkerControl()
    control_command.direction = carla.Vector3D(-1, 0, 0)
    control_command.speed = speed
    pedestrian.apply_control(control_command)



def play_scenario(world, ego_speed=7, day_time='night',
                  visibility='clear', ped_type='adult',
                  ped_limit_x_lower=PED_LIMIT_X, ego_limit_y_upper=EGO_LIMIT_Y):
    #
    # SCENARIO SETUP
    #
    # set weather and time of day
    weather_params = {**common_weather, **time_of_day[day_time], **clarity[visibility]}
    world.set_weather(carla.WeatherParameters(**weather_params))
    # spawn actors (ego, camera, parked cars, pedestrian)
    actors = []
    # EGO
    ego_init = carla.Transform(carla.Location(X_OFFSET, Y_OFFSET - 50, 0.1), carla.Rotation(0, 90, 0))
    ego = spawn_vehicle(world, ego_init, "mercedes.coupe_2020")
    actors.append(ego)
    # ego controller
    ego_cruise_control = AEBCruise(ego, target_speed=ego_speed, target_yaw=90)
    # Pedestrian
    # ped_init = carla.Transform(carla.Location(302, -194, 0.2), carla.Rotation(0,180,0))
    ped_init = carla.Transform(carla.Location(X_OFFSET+3.7, Y_OFFSET, 0.25), carla.Rotation(0, 180, 0))
    pedestrian = world.spawn_actor(pedestrian_blueprints[ped_type], ped_init)
    actors.append(pedestrian)
    # Parked cars
    parked_cars = []
    for park_init, park_model in other_parked:
        parked_npc = spawn_vehicle(world, park_init, park_model)
        parked_cars.append(parked_npc)
        actors.append(parked_npc)
    # command for parked cars (to be used in loop)
    brake_command = carla.VehicleControl()
    brake_command.brake = 1.0
    world.tick()
    # RGB Sensor
    camera = spawn_rgb_sensor(world, ego, resolution=RESOLUTION)
    actors.append(camera)
    world.get_spectator().set_transform(spectator_camera_transform(ego))
    # Collision Sensor
    collision_sensor = spawn_collision_sensor(world, ego)
    actors.append(collision_sensor)
    # Data collection
    ego_data = {'ego_x': [], 'ego_y': [], 'ego_v_x': [], 'ego_v_y': [], 'ego_a_x': [], 'ego_a_y': []}
    ped_data = {'ped_x': [], 'ped_y': [], 'ped_v_x': [], 'ped_v_y': [], 'ped_a_x': [], 'ped_a_y': []}
    time_trace = []
    #
    # SCENARIO SIMULATION
    #
    world.tick()
    initial_time = world.get_snapshot().elapsed_seconds
    simulator_time = initial_time
    carla_utils.frame_counter = -1
    carla_utils.detected_crash = False
    current_frame = 0
    ped_moved_by_ego = False
    while ego.get_location().y<ego_limit_y_upper \
            and pedestrian.get_location().x>ped_limit_x_lower \
            and not carla_utils.detected_crash\
            and not ped_moved_by_ego\
            and (simulator_time-initial_time)<EPISODE_LENGTH:
        world.tick()
        # COLLECT DATA
        simulator_time = world.get_snapshot().elapsed_seconds
        time_trace.append(round(simulator_time - initial_time, 3))
        update_actor_data(ego, ego_data, prefix='ego_', x_offset=X_OFFSET, y_offset=Y_OFFSET)
        update_actor_data(pedestrian, ped_data, prefix='ped_', x_offset=X_OFFSET, y_offset=Y_OFFSET)
        if abs(pedestrian.get_location().y - Y_OFFSET) > 0.02:
            print(f'collision detector: {carla_utils.detected_crash} , Ped offset: {pedestrian.get_location().y - Y_OFFSET:.2f}')
            ped_moved_by_ego = True
        # PROCESS FOR NEXT FRAME
        while current_frame != carla_utils.frame_counter:
            time.sleep(0.01)
        current_frame += 1
        cam_img = carla_utils.array_output
        ped_in_sight, prob = yolo_predict(cam_img)
        ego_ttc = abs((ego.get_location().y - Y_OFFSET+2.337)/(ego.get_velocity().y + 0.01))
        # print(f'Time: {simulator_time-initial_time:.2f}   Ped x {pedestrian.get_location().x - X_OFFSET:.3f}')
        #if 3<ego_ttc<4.2:
        #    print(f'Time: {simulator_time - initial_time:.2f}   Ego ttc: {ego_ttc:.3f}')
        # print(f'Pedestrian In Sight: {ped_in_sight}   Prob: {prob}')
        ego_cruise_control.control(emergency_brake=ped_in_sight)
        if ego_ttc<2.5:
            walk(pedestrian, 1.4)  # from wikipedia https://en.wikipedia.org/wiki/Preferred_walking_speed
        world.get_spectator().set_transform(spectator_camera_transform(ego))
        for parked_car in parked_cars:
            parked_car.apply_control(brake_command)
        # print(f'Ego Speed: {ego.get_velocity().length():.2f} m/s    Yaw: {ego.get_transform().rotation.yaw:.2f} ')
    camera.stop()
    collision_sensor.stop()
    for actor in actors:
        carla.command.DestroyActor(actor)
    outputs = {'crash': carla_utils.detected_crash or ped_moved_by_ego,
               'time': time_trace,
               **ego_data,
               **ped_data}
    return outputs


def save_dict_csv(input_dict, filename):
    # open file for writing, "w" is writing
    w = csv.writer(open(filename+".csv", "w"))
    for key, val in input_dict.items():
        # write every key and value to file
        w.writerow([key, val])

if __name__ == "__main__":
    #
    """ SCENARIO VARIATION OPTIONS """
    #
    from os import listdir
    day_options = ["day", "night"]
    clarity_options = ["clear", "fog"]
    ped_type_options = ["child", "adult"]
    ego_speed_options= np.arange(20, 48.2, 0.2)/3.6   # 20 to 50 kph
    # ego_speed_options = np.arange(25, 55, 5) / 3.6  # 20 to 50 kph
    options = [day_options, clarity_options, ped_type_options, ego_speed_options]
    prev_experiments = [int(f[4:][:-4]) for f in listdir('parking')]
    if prev_experiments == []:
        last_experiment = -1
    else:
        last_experiment = max(prev_experiments)
    for i, opt in enumerate(itertools.product(*options)):
        if i<=last_experiment:
            continue
        scenario_params = {'day_time': opt[0],
                           'visibility': opt[1],
                           'ped_type': opt[2],
                           'ego_speed': opt[3]}

        tr = play_scenario(world, **scenario_params)
        scenario_traces = {**scenario_params, **tr}
        #save_dict_csv(scenario_traces, f'parking_originaldata/data{i}')
        print(f'{i}: Time: {opt[0][0]}, Vis: {opt[1][0]}, Ped: {opt[2][0]},  Speed: {opt[3]:.2f}, Crash: {tr["crash"]}')
        world = client.reload_world()
        world.apply_settings(settings)


# LEGACY JUNK
"""
def accelerate(vehicle, accel):
    control_command = carla.VehicleControl()
    control_command.throttle = accel
    control_command.steer = 0.0
    vehicle.apply_control(control_command)
"""