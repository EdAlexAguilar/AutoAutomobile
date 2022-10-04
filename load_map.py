import carla

# Create CARLA world
client = carla.Client('localhost', 2000)
world = client.load_world('Town04')
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
client.reload_world()

