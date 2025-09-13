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
# ーーーー ここから追加 ーーーー
def _find_vehicle_bp(bl, preferred: str):
    """存在する車両BPを返す。preferred が無ければいくつか代替を試す。"""
    candidates = [
        preferred,
        "vehicle.audi.tt",
        "vehicle.audi.a2",
        "vehicle.lincoln.mkz_2020",
        "vehicle.tesla.model3",
        "vehicle.nissan.micra",
    ]
    for name in candidates:
        try:
            return bl.find(name)
        except RuntimeError:
            continue
    # 最後の手段：一番最初に見つかった車両
    for bp in bl.filter("vehicle.*"):
        return bp
    raise RuntimeError("No vehicle blueprint found.")

def spawn_npc_ahead(world, bl, ego, distance_m: float, autopilot: bool, z_offset: float):
    """
    自車の進行方向に、同じレーン上で distance_m だけ前に NPC をスポーン。
    レーンに沿った向きで配置されるので安全です。
    """
    amap = world.get_map()
    ego_wp = amap.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)

    # 複数距離を試して空いている場所に置く（衝突回避）
    distances = [distance_m, distance_m + 3.0, distance_m + 6.0, 10.0]
    npc = None
    npc_bp = _find_vehicle_bp(bl, config.NPC_MODEL)

    for d in distances:
        next_wps = ego_wp.next(d)
        if not next_wps:
            continue
        tr = next_wps[0].transform
        tr.location.z += z_offset
        npc = world.try_spawn_actor(npc_bp, tr)
        if npc is not None:
            break

    if npc is None:
        raise RuntimeError("Failed to spawn NPC ahead. (No free space)")

    # オートパイロット設定（False の場合はその場で停止）
    if autopilot:
        npc.set_autopilot(True)
    else:
        npc.set_autopilot(False)
        npc.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))

    return npc
# ーーーー 追加ここまで ーーーー