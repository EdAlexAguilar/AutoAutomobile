import carla
import numpy as np

class PurePursuit():
    """
    Waypoints are ordered (carla.Waypoint objects)
    """
    def __init__(self, vehicle, waypoints, world, lookahead=50):
        self.vehicle = vehicle
        self.waypoints = waypoints # list of Vector3D
        self.world = world # carla.world
        self.lookahead = lookahead
        self.Kpp = 5
        self.half_length = vehicle.bounding_box.extent.x  - 0.5 # half the vehicle's length correction to put in axle
        self.draw_waypoints()

    def draw_waypoints(self):
        for i in range(len(self.waypoints)-1):
            self.world.debug.draw_line(self.waypoints[i], self.waypoints[i+1], color=carla.Color(0,100,0), life_time=0)

    def append_waypoints(self, new_waypoints):
        self.waypoints = self.waypoints + new_waypoints

    def update(self):
        speed = self.vehicle.get_velocity()
        speed = speed.length()
        # lookahead = self.lookahead # Naive Version
        lookahead = self.Kpp*speed  # Updated version with speed dependence
        control_command = carla.VehicleControl()
        if speed<(50/3.6):
            # ridiculous speed controller
            control_command.throttle = 0.6
        else:
            control_command.throttle = 0.4
        transform = self.vehicle.get_transform()
        location = transform.location
        heading = transform.rotation.get_forward_vector()
        rear_axle_loc = location - self.half_length*heading
        distances = np.array([(rear_axle_loc.distance(w) - lookahead) for w in self.waypoints])
        target_id = np.abs(distances).argmin()
        self.waypoints = self.waypoints[target_id:]
        v_target = self.waypoints[0] - rear_axle_loc
        v_target_length = v_target.length()
        # assert abs(v_target_length - self.lookahead) < 8 , "Target Waypoint is not at desired lookahead dist"
        v_target_unitary = v_target.make_unit_vector()
        sin_alpha = v_target_unitary.cross(heading)
        sign_sin_alpha = np.sign(sin_alpha.dot(carla.Vector3D(z=1)))
        sin_alpha = sin_alpha.length()*sign_sin_alpha
        delta = - np.arctan(4*self.half_length*sin_alpha / v_target_length)
        delta *= 2/np.pi
        control_command.steer = delta
        # control_command.steer = 0.5 * (2*np.random.rand() - 1)
        self.vehicle.apply_control(control_command)

        """
        Helpful Drawing of Lines
        """
        higher_rear_axle = rear_axle_loc
        higher_rear_axle.z += 0.4
        self.world.debug.draw_arrow(higher_rear_axle, self.waypoints[0], thickness=0.2, color=carla.Color(0,0,50), life_time=0.08)
        self.world.debug.draw_point(rear_axle_loc, size=0.05, color=carla.Color(0,0,0,50), life_time=0)


        pass