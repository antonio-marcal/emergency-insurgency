import carla

def destroy_actor(blueprint):
    client = carla.Client('localhost', 2000)
    world=client.get_world()
    vehicles=world.get_actors().filter(blueprint)
    for v in vehicles:
        v.destroy()


