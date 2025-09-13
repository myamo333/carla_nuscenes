import os
import numpy as np
import struct
from tqdm import tqdm

# ===== PCD ヘッダ（nuScenes radar と完全一致）=====
_PCD_HEADER = (
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\n"
    "FIELDS x y z dyn_prop id rcs vx vy vx_comp vy_comp "
    "is_quality_valid ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms\n"
    "SIZE 4 4 4 1 2 4 4 4 4 4 1 1 1 1 1 1 1 1\n"
    # nuScenes 実ファイルと同じ TYPE（U ではなく I）
    "TYPE F F F I I F F F F F I I I I I I I I\n"
    "COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n"
    "WIDTH {n}\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS {n}\n"
    "DATA binary\n"
)

# 1点のパック形式（合計 43 bytes/point）
# <fff  b   h   f   f   f   f   f   b  b  b  b  b  b  b  b
#  x,y,z dyn id  rcs vx  vy  vx_c vy_c iq as xr yr inv pd vxr vyr
_PACK = struct.Struct('<fff b h f f f f f b b b b b b b b').pack


def _carla_polar_to_nuscenes_xyz(depth, az, alt):
    """
    CARLA radar の極座標を nuScenesのセンサ座標系(X前+, Y左+, Z上+)の直交座標へ。
    CARLA局所: x前+, y右+, z上+ → yを反転して左+にする。
    """
    x = depth * np.cos(alt) * np.cos(az)
    y_right = depth * np.cos(alt) * np.sin(az)
    z = depth * np.sin(alt)
    y = -y_right  # 左+へ変換
    return x.astype(np.float32), y.astype(np.float32), z.astype(np.float32)


def convert_bin_to_nuscenes_pcd(bin_path: str, pcd_path: str) -> None:
    """
    Carla radar .bin (float32: depth, azimuth[rad], altitude[rad], velocity[m/s])
      -> nuScenes互換 PCD(binary, 18 fields)
    """
    scan = np.fromfile(bin_path, dtype=np.float32)
    if scan.size == 0:
        # 空でもヘッダは正規に作っておく
        with open(pcd_path, 'wb') as f:
            f.write(_PCD_HEADER.format(n=0).encode('ascii'))
            f.write(b'\n')
        return
    if scan.size % 4 != 0:
        raise ValueError(f"{bin_path}: float32の数が4の倍数ではありません（{scan.size}）")

    pts = scan.reshape(-1, 4)
    depth, az, alt, vel = pts.T

    # 位置
    x, y, z = _carla_polar_to_nuscenes_xyz(depth, az, alt)
    n = x.shape[0]

    # 速度（視線速度を平面へ簡易投影。yは左+系に合わせて符号反転）
    vx = (vel * np.cos(az) * np.cos(alt)).astype(np.float32)
    vy = (-vel * np.sin(az) * np.cos(alt)).astype(np.float32)
    vx_comp = vx.copy()
    vy_comp = vy.copy()

    # 残りのフィールド（最低限の妥当値）
    dyn_prop = np.zeros(n, dtype=np.int8)        # 0:moving 相当
    ids      = np.arange(n, dtype=np.int16)
    rcs      = np.zeros(n, dtype=np.float32)
    is_quality_valid = np.ones(n, dtype=np.int8) # 1=valid
    ambig_state      = np.full(n, 3, dtype=np.int8)  # 3=unambiguous（既定フィルタ通過）
    x_rms = y_rms = invalid_state = pdh0 = vx_rms = vy_rms = np.zeros(n, dtype=np.int8)

    # 出力
    with open(pcd_path, 'wb') as f:
        f.write(_PCD_HEADER.format(n=n).encode('ascii'))
        for i in range(n):
            f.write(_PACK(
                float(x[i]), float(y[i]), float(z[i]),
                int(dyn_prop[i]), int(ids[i]), float(rcs[i]),
                float(vx[i]), float(vy[i]), float(vx_comp[i]), float(vy_comp[i]),
                int(is_quality_valid[i]), int(ambig_state[i]),
                int(x_rms[i]), int(y_rms[i]), int(invalid_state[i]),
                int(pdh0[i]), int(vx_rms[i]), int(vy_rms[i])
            ))
        # nuScenesの読み出し実装が < を使う環境があるため、末尾に1バイト追加
        f.write(b'\n')


def batch_convert_radar(bin_root: str) -> None:
    """
    bin_root 配下の RADAR_* ディレクトリにある .bin を .pcd へ変換（同ディレクトリに出力）。
    """
    for root, _, files in os.walk(bin_root):
        rel = os.path.relpath(root, bin_root)
        parts = [p for p in rel.split(os.sep) if p and p != '.']
        if not any(p.startswith("RADAR_") for p in parts):
            continue
        targets = [f for f in files if f.endswith(".bin")]
        for fname in tqdm(targets, desc=f"Converting {rel}"):
            src = os.path.join(root, fname)
            dst = os.path.join(root, fname[:-4] + ".pcd")
            convert_bin_to_nuscenes_pcd(src, dst)


if __name__ == "__main__":
    base = "./data/nuscenes_eval"
    for sub in ("sweeps", "samples"):
        d = os.path.join(base, sub)
        if os.path.isdir(d):
            print(f"▶ RADAR_* in {sub}: .bin → .pcd")
            batch_convert_radar(d)
    print("✅ RADAR_* の .bin → .pcd 変換完了（nuScenes完全互換・Y反転・アンビギュイティ対策・末尾1Bパディング）")
