import carla

class PurePursuit():
    """
    Waypoints are ordered (carla.Waypoint objects)
    """
    def __init__(self, vehicle, waypoints):
        self.vehicle = vehicle
        self.waypoints = waypoints

    def update(self):
        control_command = carla.VehicleControl()
        control_command.throttle = 0.5

        self.vehicle.apply_control(control_command)
        pass