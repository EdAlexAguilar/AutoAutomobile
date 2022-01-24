import carla
import numpy as np

class PurePursuit():
    """
    Waypoints are ordered (carla.Waypoint objects)
    """
    def __init__(self, vehicle, waypoints, world, kpp=5, target_speed_kph=45):
        self.vehicle = vehicle
        self.waypoints = waypoints # list of Vector3D
        self.world = world # carla.world
        self.kpp = kpp
        self.target_speed_kph = target_speed_kph
        self.half_length = vehicle.bounding_box.extent.x  - 0.5 # half the vehicle's length correction to put in axle
        self.draw_waypoints()

    def draw_waypoints(self):
        for i in range(len(self.waypoints)-1):
            self.world.debug.draw_line(self.waypoints[i], self.waypoints[i+1], color=carla.Color(0,10,0), life_time=0)

    def append_waypoints(self, new_waypoints):
        self.waypoints = self.waypoints + new_waypoints

    def longitudinal_control(self):
        # ridiculous speed controller
        speed = self.vehicle.get_velocity()
        speed = speed.length()
        if speed < (self.target_speed_kph):
            throttle = 0.6
        else:
            throttle = 0.4
        return throttle

    def lateral_control(self):
        speed = self.vehicle.get_velocity()
        speed = speed.length()
        lookahead = self.kpp * speed

        transform = self.vehicle.get_transform()
        location = transform.location
        heading = transform.rotation.get_forward_vector()
        rear_axle_loc = location - self.half_length * heading

        distances = np.array([(rear_axle_loc.distance(w) - lookahead) for w in self.waypoints])
        target_id = np.abs(distances).argmin()
        self.waypoints = self.waypoints[target_id:]
        v_target = self.waypoints[0] - rear_axle_loc
        v_target_length = v_target.length()

        v_target_unitary = v_target.make_unit_vector()
        sin_alpha = v_target_unitary.cross(heading)
        sign_sin_alpha = np.sign(sin_alpha.dot(carla.Vector3D(z=1)))
        sin_alpha = sin_alpha.length() * sign_sin_alpha
        delta = - np.arctan(4 * self.half_length * sin_alpha / v_target_length)
        delta *= 2 / np.pi

        """
        Helpful Drawing of Lines
        """
        higher_rear_axle = rear_axle_loc
        higher_rear_axle.z += 0.4
        self.world.debug.draw_arrow(higher_rear_axle, self.waypoints[0],
                                    thickness=0.2, color=carla.Color(0, 0, 50), life_time=0.08)
        self.world.debug.draw_point(rear_axle_loc,
                                    size=0.05, color=carla.Color(0, 50, 50, 50), life_time=0)
        return delta

    def update(self):
        control_command = carla.VehicleControl()
        control_command.throttle = self.longitudinal_control()
        control_command.steer = self.lateral_control()
        self.vehicle.apply_control(control_command)
