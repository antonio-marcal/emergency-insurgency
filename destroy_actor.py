import carla

def destroy_actor(blueprint, world):
    vehicles=world.get_actors().filter(blueprint)
    for v in vehicles:
        v.destroy()

def move_slow_vehicle(blueprint, world):
    vehicles=world.get_actors().filter(blueprint)
    for v in vehicles:
        v.set_autopilot(True)


if __name__ == "__main__":
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Create Traffic Manager instance
    traffic_manager = client.get_trafficmanager()

    # Set Traffic Manager properties (you can adjust these as needed)
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)
    traffic_manager.set_hybrid_physics_mode(False)
    
    move_slow_vehicle("*citroen.c3*", world)

    while True:
        pass
