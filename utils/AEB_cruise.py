from utils.pid import PID
import carla

class AEBCruise:
    def __init__(self, ego_vehicle, target_speed=0, target_yaw=0,
                 speed_kp=0.38, speed_kd=0.2, speed_ki=0,
                 steer_kp=0.08, steer_kd=0.1, steer_ki=0):
        self.ego = ego_vehicle # Carla Vehicle object
        self.target_speed = target_speed # in m/s
        self.target_yaw = target_yaw  # in absolute world coordinates
        self.speed_controller = PID(kp=speed_kp, kd=speed_kd, ki=speed_ki, base_control=0.4)
        self.steer_controller = PID(kp=steer_kp, kd=steer_kd, ki=steer_ki, base_control=0)
        self.previous_brake = False

    def control(self, emergency_brake=False):
        """
        Function has side-effect of applying vehicle control to self.ego
        """
        measured_speed = self.ego.get_velocity().length()
        measured_yaw = self.ego.get_transform().rotation.yaw
        speed_control = self.speed_controller.control(self.target_speed, measured_speed)
        steer_control = self.steer_controller.control(self.target_yaw, measured_yaw)
        control_command = carla.VehicleControl()
        if emergency_brake:
            self.previous_brake = True
        if self.previous_brake:
            control_command.throttle = 0
            control_command.brake = 1.0
        else:
            control_command.throttle = speed_control
        control_command.steer = steer_control
        self.ego.apply_control(control_command)