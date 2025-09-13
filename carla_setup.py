import carla
import config

def init_world():
    client = carla.Client(config.CARLA_HOST, config.CARLA_PORT)
    client.set_timeout(10.0)
    world = client.load_world(config.TOWN)
    world.set_weather(carla.WeatherParameters.ClearNoon)
    bl = world.get_blueprint_library()
    return client, world, bl

def spawn_vehicle(world, bl):
    prius_bp = bl.find("vehicle.toyota.prius")
    spawn = world.get_map().get_spawn_points()[0]
    prius = world.spawn_actor(prius_bp, spawn)
    return prius
