import carla

import random


def spawn_specific_vehicle(world, blueprint_library, vehicle_type, location, rotation=(0, 0, 0)):

   """

   Spawns a specific vehicle at the given location and rotation.


   Args:

       world: The CARLA world object.

       blueprint_library: The blueprint library from CARLA.

       vehicle_type: The type of the vehicle (e.g., "vehicle.audi.tt").

       location: A tuple (x, y, z) specifying the spawn location.

       rotation: A tuple (pitch, yaw, roll) specifying the spawn rotation.

   """

   blueprint = blueprint_library.find(vehicle_type)

   if blueprint is None:

       print(f"Vehicle type '{vehicle_type}' not found.")

       return None


   transform = carla.Transform(

       carla.Location(x=location[0], y=location[1], z=location[2]),

       carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])

   )

   vehicle = world.try_spawn_actor(blueprint, transform)

   if vehicle:

       if __name__ == "__main__":
            print(f"Spawned {vehicle.type_id} at location {location} with rotation {rotation}.")

       return vehicle

   else:

       print(f"Failed to spawn {vehicle_type} at location {location}.")

       return None


def main(world = None):

   # Connect to the Carla Simulator

   if world is None:
    client = carla.Client('localhost', 2000)  # Default CARLA port
    client.set_timeout(10.0)
    world = client.get_world()


    # Set the world to nighttime

   weather = carla.WeatherParameters(cloudiness=00.0,precipitation=00.0,sun_altitude_angle=10.0)


   world.set_weather(weather)



   # Get the blueprint library

   blueprint_library = world.get_blueprint_library()

   vehicle_blueprints = blueprint_library.filter('vehicle.*')



   num_vehicles = 2

   spawn_points = [carla.Transform(carla.Location(x=-70.0, y=28.0, z=0.6), carla.Rotation(pitch=0.000000, yaw=0.0, roll=0.000000)),

                   carla.Transform(carla.Location(x=-80.0, y=28.0, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.0, roll=0.000000)),

   ]

   for i in range(num_vehicles):

       blueprint = random.choice(vehicle_blueprints)

       vehicle = world.try_spawn_actor(blueprint, spawn_points[i-1])


   num_vehicles2 = 5

   spawn_points2 = [carla.Transform(carla.Location(x=109.4, y=82.6, z=0.6), carla.Rotation(pitch=0.000000, yaw=-90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=109.4, y=87.6, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=109.4, y=92.6, z=0.6), carla.Rotation(pitch=0.000000, yaw=-90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=109.4, y=97.6, z=0.6), carla.Rotation(pitch=0.000000, yaw=-90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=109.4, y=102.6, z=0.6), carla.Rotation(pitch=0.000000, yaw=-90.0, roll=0.000000)),

             

   ]

   for i in range(num_vehicles2):

       blueprint = random.choice(vehicle_blueprints)

       vehicle = world.try_spawn_actor(blueprint, spawn_points2[i-1])


   num_vehicles3 = 5

   spawn_points3 = [carla.Transform(carla.Location(x=99.1, y=-8.4, z=0.6), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=99.1, y=-13.4, z=0.6), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=99.1, y=-18.4, z=0.6), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=99.1, y=-23.4, z=0.6), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000)),

                    carla.Transform(carla.Location(x=99.1, y=-28.4, z=0.6), carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000)),    

   ]

   for i in range(num_vehicles3):

       blueprint = random.choice(vehicle_blueprints)

       vehicle = world.try_spawn_actor(blueprint, spawn_points3[i-1])

       

  


   # Example: Spawn a specific vehicle at a specific location

   specific_vehicle_type = 'vehicle.dodge.charger_police'  # Example vehicle type

   spawn_location = (-42.4, 28.3, 2.0)  # Example location (x, y, z)

   spawn_rotation = (0, 0, 0)  # Example rotation (pitch, yaw, roll)


   # Spawn the specific vehicle

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type, spawn_location, spawn_rotation)


   # Additional spawns can be added here...

   specific_vehicle_type2 = 'vehicle.dodge.charger_police'  # Example vehicle type

   spawn_location2 = (-52.3, 28.3, 2.0)  # Example location (x, y, z)

   spawn_rotation2 = (0, 0, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type2, spawn_location2, spawn_rotation2)


   specific_walker_type = 'walker.pedestrian.0030'  # Example vehicle type

   spawn_location3 = (-42.4, 33.0, 2.0)  # Example location (x, y, z)

   spawn_rotation3 = (0, 90, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_walker_type, spawn_location3, spawn_rotation3)


   # Additional spawns can be added here...

   specific_vehicle_type3 = 'vehicle.carlamotors.firetruck'  # Example vehicle type

   spawn_location4 = (-47.6, 20, 5.0)  # Example location (x, y, z)

   spawn_rotation4 = (0, 0, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type3, spawn_location4, spawn_rotation4)


   specific_vehicle_type4 = 'vehicle.citroen.c3'  # Overtaken Car

   spawn_location5 = (16, 69.9, 2.0)  # Example location (x, y, z)

   spawn_rotation5 = (0, 0, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type4, spawn_location5, spawn_rotation5)


   specific_vehicle_type5 = 'vehicle.dodge.charger_police'  # Example vehicle type

   spawn_location6 = (107.8, 74.2, 2.0)  # Example location (x, y, z)

   spawn_rotation6 = (0, 0, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type5, spawn_location6, spawn_rotation6)


   specific_walker_type2 = 'walker.pedestrian.0036'  # Victim

   spawn_location7 = (108.2, 26.3, 2.0)  # Example location (x, y, z)

   spawn_rotation7 = (90, 90, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_walker_type2, spawn_location7, spawn_rotation7)


   specific_walker_type3 = 'walker.pedestrian.0031'  # Example vehicle type

   spawn_location8 = (110.2, 26.3, 2.0)  # Example location (x, y, z)

   spawn_rotation8 = (0, 180, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_walker_type3, spawn_location8, spawn_rotation8)


   specific_walker_type4 = 'walker.pedestrian.0032'  # Example vehicle type

   spawn_location9 = (110.2, 28.3, 2.0)  # Example location (x, y, z)

   spawn_rotation9 = (0, 180, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_walker_type4, spawn_location9, spawn_rotation9)


   specific_walker_type5 = 'walker.pedestrian.0015'  # Example vehicle type

   spawn_location10 = (110.2, 24.3, 2.0)  # Example location (x, y, z)

   spawn_rotation10 = (0, 180, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_walker_type5, spawn_location10, spawn_rotation10)


   specific_vehicle_type6 = 'vehicle.dodge.charger_police'  # Example vehicle type

   spawn_location11 = (99.9, 3.1, 2.0)  # Example location (x, y, z)

   spawn_rotation11 = (0, 0, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type6, spawn_location11, spawn_rotation11)


   specific_vehicle_type7 = 'vehicle.ford.mustang'  # Example vehicle type

   spawn_location12 = (-36.7, 19.0, 2.0)  # Example location (x, y, z)

   spawn_rotation12 = (0, -135, 180)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type7, spawn_location12, spawn_rotation12)


   specific_vehicle_type8 = 'vehicle.dodge.charger_police'  # Example vehicle type

   spawn_location13 = (-65.2, 28.4, 2.0)  # Example location (x, y, z)

   spawn_rotation13 = (0, 90, 0)  # Example rotation (pitch, yaw, roll)

   spawn_specific_vehicle(world, blueprint_library, specific_vehicle_type8, spawn_location13, spawn_rotation13)

if __name__ == '__main__':

   main()