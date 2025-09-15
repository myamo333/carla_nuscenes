import time
import os
from PIL import Image
import config
from utils import make_directory
from carla_setup import init_world, spawn_vehicle, spawn_npc_ahead   # ★ 追加
from sensors import attach_cameras, attach_radars, attach_lidar
from timeline import compute_sample_times, pick_keyframes_and_copy
from nuscenes_writer import write_nuscenes_jsons
import carla

def ensure_dirs():
    samples_dir = os.path.join(config.BASE_DIR, "samples")
    sweeps_dir  = os.path.join(config.BASE_DIR, "sweeps")
    maps_dir    = os.path.join(config.BASE_DIR, "maps")
    for d in [samples_dir, sweeps_dir, maps_dir]:
        make_directory(d)
    # 画像/レーダー/LIDARの各チャンネルのsamplesディレクトリも先に作成
    for name in config.CAM_NAMES + config.RADAR_NAMES + [config.LIDAR_NAME]:
        make_directory(os.path.join(samples_dir, name))
        make_directory(os.path.join(sweeps_dir, name))
    # ダミーマップ
    Image.new('RGB', (1, 1), color=(0,0,0)).save(os.path.join(maps_dir, "eval_map.png"))
    return samples_dir, sweeps_dir

def main():
    # CARLA
    client, world, bl = init_world()
    prius = spawn_vehicle(world, bl)
    ego_spawn_tf = prius.get_transform() 

    # ★ 前方NPCを必要ならスポーン
    npc_actor = None
    pedestrian = None
    blueprint_library = world.get_blueprint_library()
    pedestrian_bp = blueprint_library.find('walker.pedestrian.0003')
    spawn_points_ped = carla.Transform(carla.Location(x=-6.45, y=157.19, z=1), carla.Rotation(yaw=0))
    # pedestrian = world.spawn_actor(pedestrian_bp, spawn_points_ped)
    pedestrian = world.try_spawn_actor(pedestrian_bp, spawn_points_ped)
    if getattr(config, "NPC_ENABLED", False):
        npc_actor = spawn_npc_ahead(
            world, bl, prius,
            distance_m=config.NPC_AHEAD_METERS,
            autopilot=config.NPC_AUTOPILOT,
            z_offset=getattr(config, "NPC_SPAWN_Z_OFFSET", 0.5),
        )
        loc = npc_actor.get_transform().location
        # print(f"[EGO] spawned at x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}")

    samples_dir, sweeps_dir = ensure_dirs()

    # センサー
    cam_actors, captured_images = attach_cameras(world, bl, prius, sweeps_dir)
    radar_actors, captured_radar = attach_radars(world, bl, prius, sweeps_dir)
    lidar_actor, captured_lidar = attach_lidar(world, bl, prius, sweeps_dir)

    # 走行＆撮影
    prius.set_autopilot(True)
    time.sleep(config.DURATION_SEC)

    # サンプル時刻
    sample_times = compute_sample_times(captured_images, captured_lidar)

    # keyframesコピー
    key_img_for_idx   = pick_keyframes_and_copy(captured_images, sample_times, sweeps_dir, samples_dir)
    key_radar_for_idx = pick_keyframes_and_copy(captured_radar, sample_times, sweeps_dir, samples_dir)
    key_lidar_for_idx = {}
    if captured_lidar:
        # LIDAR_TOPについては各idxごとに最も近いものを samples/ にコピー
        from timeline import pick_keyframes_and_copy as _pick
        # 便宜的にdict化して再利用
        _tmp = {"LIDAR_TOP": captured_lidar}
        copied = _pick(_tmp, sample_times, sweeps_dir, samples_dir)
        key_lidar_for_idx = {idx: src for idx, src in copied["LIDAR_TOP"].items()}

    # JSON出力
    write_nuscenes_jsons(
        base_dir=config.BASE_DIR,
        sample_times=sample_times,
        key_img_for_idx=key_img_for_idx,
        key_radar_for_idx=key_radar_for_idx,
        key_lidar_for_idx=key_lidar_for_idx,
        captured_images=captured_images,
        captured_radar=captured_radar,
        captured_lidar=captured_lidar
    )

    print("✅ NuScenes形式の出力が完了しました。")

    # 後片付け
    for a in cam_actors: a.destroy()
    for a in radar_actors: a.destroy()
    if lidar_actor: lidar_actor.destroy()
    if npc_actor: npc_actor.destroy()     # ★ 追加
    prius.destroy()

if __name__ == "__main__":
    main()
