import yaml


class WaypointManager:
    def __init__(self, yaml_file):
        self.yaml_file = yaml_file
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        with open(self.yaml_file, 'r') as file:
            return yaml.safe_load(file)

    def get_waypoint(self, name):
        return self.waypoints.get(name, None)

    def update_waypoint(self, name, x, y, yaw):
        self.waypoints[name] = {
            'x': x,
            'y': y,
            'yaw': yaw
        }

    def save_waypoints(self):
        with open(self.yaml_file, 'w') as file:
            yaml.dump(self.waypoints, file)
