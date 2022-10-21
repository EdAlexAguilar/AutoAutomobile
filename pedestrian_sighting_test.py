import carla
import numpy as np
from utils.carla_utils import spawn_vehicle, spectator_camera_transform, spawn_rgb_sensor, spawn_collision_sensor
import utils.carla_utils as carla_utils
from utils.AEB_cruise import AEBCruise
import time
from yolov7.main_detector import YOLODetector
import csv
import itertools
from PIL import Image


client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
# carla_map = world.get_map()
# EPISODE ENDING CONDITIONS
EPISODE_LENGTH = 20
PED_LIMIT_X = 281 # lower
EGO_LIMIT_Y = -185 # upper
# COORDINATE OFFSETS
X_OFFSET = 286
Y_OFFSET = -193.5

# LOCATION OF OTHER PARKED VEHICLES, JUST FOR VISUALIZATION PURPOSES
other_parked = [(carla.Transform(carla.Location(280, -201, 0.25), carla.Rotation(0, 188, 0)),'volkswagen.t2_2021', "50,150,180"),
                (carla.Transform(carla.Location(290.2, -201.5, 0.22)),'nissan.patrol', "170,170,190"),
                (carla.Transform(carla.Location(289.7, -198, 0.22), carla.Rotation(0, 185, 0)),'seat.leon', "10,10,0"),
                (carla.Transform(carla.Location(290.2, -191, 0.22), carla.Rotation(0, -10, 0)),'audi.etron', "120,30,30"),
                (carla.Transform(carla.Location(298, -201, 0.2), carla.Rotation(0, 177, 0)),'jeep.wrangler_rubicon', "25,100,65"),
                (carla.Transform(carla.Location(280.5, -191.5, 0.3), carla.Rotation(0, 188, 0)),'mini.cooper_s',"80,180,100"),
                (carla.Transform(carla.Location(280.9, -195, 0.28), carla.Rotation(0, -5, 0)),'toyota.prius', "40,30,65")]

# WEATHER AND TIME OF DAY OPTIONS
common_weather = {'sun_azimuth_angle': 80}
night = {'sun_altitude_angle': -3}  # dusk
day = {'sun_altitude_angle': 30,
       'cloudiness': 70}  # afternoon, sun in horizon
clear = {}
fog = {'precipitation_deposits': 70,
       'fog_density': 100,
       'cloudiness': 100,
       'scattering_intensity': 100}
time_of_day = {'day': day, 'night': night}
clarity = {'clear': clear, 'fog': fog}
# PEDESTRIAN BLUEPRINTS
pedestrian_blueprints = {'child': world.get_blueprint_library().find(f'walker.pedestrian.0012'),
                         'adult': world.get_blueprint_library().find(f'walker.pedestrian.0028')}

# SENSOR & YOLO  (Initial loading is slow - so only do it once)
RESOLUTION = 1280
perception_model = YOLODetector(img_size=RESOLUTION, conf_threshold=0.1)


def yolo_predict(image):
    prediction = perception_model.detect(image)
    pred = prediction[0].detach().cpu().numpy()
    if pred.shape[0] == 0:
        pred = np.zeros((1, 6))
        pred[0, -1] = -1  # to make sure that class is not accidently considered '0'
    # Check if a pedestrian was observed or not
    person_probs = np.array([pred[i][-2] if pred[i][-1]==0 else 0.0 for i in range(pred.shape[0])])
    if 0 in pred[:,-1]:
        return True, max(person_probs)
    else:
        return False, max(person_probs)


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
                  ped_limit_x_lower=PED_LIMIT_X, ego_limit_y_upper=EGO_LIMIT_Y,
                  im_save=False):
    #
    # SCENARIO SETUP
    #
    # set weather and time of day
    weather_params = {**common_weather, **time_of_day[day_time], **clarity[visibility]}
    world.set_weather(carla.WeatherParameters(**weather_params))
    # spawn actors (ego, camera, parked cars, pedestrian)
    actors = []
    # EGO
    ego_init = carla.Transform(carla.Location(X_OFFSET, Y_OFFSET - 7.5, 0.3), carla.Rotation(0, 90, 0))
    ego = spawn_vehicle(world, ego_init, "mercedes.coupe_2020", "25,15,25")
    actors.append(ego)
    # ego controller
    ego_cruise_control = AEBCruise(ego, target_speed=ego_speed, target_yaw=90)
    if day_time == "night":
        ego.set_light_state(carla.VehicleLightState.LowBeam)
    # Pedestrian
    # ped_init = carla.Transform(carla.Location(302, -194, 0.2), carla.Rotation(0,180,0))
    ped_init = carla.Transform(carla.Location(X_OFFSET+4.0, Y_OFFSET, 0.25), carla.Rotation(0, 180, 0))
    pedestrian = world.spawn_actor(pedestrian_blueprints[ped_type], ped_init)
    actors.append(pedestrian)
    # Parked cars
    parked_cars = []
    for park_init, park_model, park_color in other_parked:
        parked_npc = spawn_vehicle(world, park_init, park_model, park_color)
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
    detection_prob_trace = []
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
    first_sighting = True
    first_hc_sighting = True
    output_data = {'first_sight_dist': -1}
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
            # print(f'collision detector: {carla_utils.detected_crash} , Ped offset: {pedestrian.get_location().y - Y_OFFSET:.2f}')
            ped_moved_by_ego = True
        # PROCESS FOR NEXT FRAME
        while current_frame != carla_utils.frame_counter:
            time.sleep(0.01)
        current_frame += 1
        cam_img = carla_utils.array_output
        ped_in_sight, prob = yolo_predict(cam_img)

        detection_prob_trace.append(prob)

        ego_dist = abs(ego.get_location().y - Y_OFFSET + 2.337)
        if prob>0.94 and first_hc_sighting:
            print(f'{current_frame}::   Prob:{int(prob*100)}   Ped_x: {pedestrian.get_location().x-X_OFFSET:.3f}')
            first_hc_sighting = False
        if ped_in_sight and first_sighting:
            output_data['first_sight_dist'] = ego_dist
            first_sighting = False
        if im_save and prob>0.7 and current_frame%4==0:
            im = Image.fromarray(cam_img)
            im.save(f'yolo_test/{visibility}{day_time}{ped_type}_frame{current_frame}_prob{int(prob*100)}.jpg')
        ego_cruise_control.control(emergency_brake=ped_in_sight)
        if ego_dist<9+ego_speed*0.9:
            walk(pedestrian, 1.5)  # from wikipedia https://en.wikipedia.org/wiki/Preferred_walking_speed
        world.get_spectator().set_transform(spectator_camera_transform(ego))
        for parked_car in parked_cars:
            parked_car.apply_control(brake_command)
        # print(f'Ego Speed: {ego.get_velocity().length():.2f} m/s    Yaw: {ego.get_transform().rotation.yaw:.2f} ')
    camera.stop()
    collision_sensor.stop()
    for actor in actors:
        carla.command.DestroyActor(actor)
    output_data['crash'] = carla_utils.detected_crash or ped_moved_by_ego
    output_data['time'] = time_trace
    output_data['prob_trace'] = detection_prob_trace
    output_data = {**output_data,
                    **ego_data,
                    **ped_data}
    return output_data


if __name__ == "__main__":
    #
    """ SCENARIO VARIATION OPTIONS """
    #
    import matplotlib.pyplot as plt
    scenario_params = {'day_time': 'night',
                       'visibility': 'fog',
                       'ped_type': 'adult',
                       'ego_speed': 0.0}

    tr = play_scenario(world, **scenario_params, im_save=True)
    # Plot yolo probs
    ped_pos_a = -np.array(tr['ped_x'])
    prob_yolo_a = tr['prob_trace']
    world = client.reload_world()
    world.apply_settings(settings)

    scenario_params['ped_type']='child'
    tr = play_scenario(world, **scenario_params, im_save=True)

    ped_pos_c = -np.array(tr['ped_x'])
    prob_yolo_c = tr['prob_trace']

    plt.plot(ped_pos_a, prob_yolo_a, 'b.')
    plt.plot(ped_pos_c, prob_yolo_c, 'k.')
    plt.axis([-4, 4, 0.75, 1])
    plt.ylabel('Detection Probability')
    plt.xlabel('Pedestrian Position')
    plt.savefig('yolo_ped_probability.png', bbox_inches='tight')
    # plt.show()
    world = client.reload_world()
    world.apply_settings(settings)

