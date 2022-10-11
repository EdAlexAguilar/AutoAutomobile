import carla


# Create CARLA world
client = carla.Client('localhost', 2000)
client.reload_world()
world = client.load_world('Town04')
#settings = world.get_settings()
#settings.synchronous_mode = True
#settings.fixed_delta_seconds = 0.05
#world.apply_settings(settings)
#client.reload_world()

"""
child_bp = world.get_blueprint_library().find(f'walker.pedestrian.0010')
adult_bp = world.get_blueprint_library().find(f'walker.pedestrian.0028')
child_init = carla.Transform(carla.Location(285, -240, 0.45), carla.Rotation(0,180,0))
adult_init = carla.Transform(carla.Location(285, -238, 0.45), carla.Rotation(0,180,0))
child = world.spawn_actor(child_bp, child_init)
adult = world.spawn_actor(adult_bp, adult_init)
"""
"""
# spawns and lines up pedestrians 10 to 40
for x in range(38):
    ped_init = carla.Transform(carla.Location(285, -240+x, 0.45), carla.Rotation(0,180,0))
    ped_blueprint = world.get_blueprint_library().find(f'walker.pedestrian.00{10+x}')
    pedestrian = world.spawn_actor(ped_blueprint, ped_init)
    print(f'{x} spawned')
"""