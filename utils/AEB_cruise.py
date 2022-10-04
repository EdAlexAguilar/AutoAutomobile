from pid import PID
import carla

class AEBCruise:
    def __init__(self, ego_vehicle, camera, target_speed=0, target_yaw=0,
                 speed_kp=0.5, speed_kd=0.2, speed_ki=0,
                 steer_kp=0.1, steer_kd=0.1, steer_ki=0):
        self.ego = ego_vehicle # Carla Vehicle object
        self.camera = camera # Carla Camera RGB Actor
        self.target_speed = target_speed # in m/s
        self.target_yaw = target_yaw  # in absolute world coordinates
        self.speed_controller = PID(kp=speed_kp, kd=speed_kd, ki=speed_ki, base_control=0.5)
        self.steer_controller = PID(kp=steer_kp, kd=steer_kd, ki=steer_ki, base_control=0)

    def control(self):
        """
        Function has side-effect of applying vehicle control to self.ego
        """
        measured_speed = 1
        measured_yaw = 1
        speed_control = self.speed_controller.control(self.target_speed, measured_speed)
        steer_control = self.steer_controller.control(self.target_yaw, measured_yaw)
        control_command = carla.VehicleControl()
        control_command.throttle = speed_control
        control_command.steer = steer_control
        self.ego.apply_control(control_command)