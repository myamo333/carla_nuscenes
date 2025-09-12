import os
import numpy as np
import struct
from tqdm import tqdm

# 1点のバイナリ構造（nuScenes radar PCD と完全一致: 43 bytes）
# <fff  B   H   f   f   f   f   f   B  B  B  B  B  B  B  B
#  x,y,z dyn id  rcs vx  vy  vx_c vy_c iq as xr yr inv pd vxr vyr
_STRUCT_PT = struct.Struct('<fffB H f f f f f B B B B B B B B')
_POINT_STEP = _STRUCT_PT.size  # = 43

_HEADER_TEMPLATE = (
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

import os
import numpy as np
import struct
from tqdm import tqdm

# 1点のバイナリ構造（nuScenes radar PCD と完全一致: 43 bytes）
# <fff  B   H   f   f   f   f   f   B  B  B  B  B  B  B  B
#  x,y,z dyn id  rcs vx  vy  vx_c vy_c iq as xr yr inv pd vxr vyr
_STRUCT_PT = struct.Struct('<fffB H f f f f f B B B B B B B B')
_POINT_STEP = _STRUCT_PT.size  # = 43

_HEADER_TEMPLATE = (
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

def convert_bin_to_nuscenes_pcd(bin_path: str, pcd_path: str):
    """
    Carla radar .bin (depth, azimuth, altitude, velocity -> float32 x4)
    -> nuScenes互換 PCD(binary, 18フィールド)
    """
    pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    if pts.size == 0:
        # 空ファイルでもPCDの体裁は作る（幅0）
        n = 0
        x = y = z = np.zeros(0, dtype=np.float32)
    else:
        depth, azimuth, altitude, velocity = pts.T
        # 極座標 -> デカルト
        x = depth * np.cos(altitude) * np.cos(azimuth)
        y = depth * np.cos(altitude) * np.sin(azimuth)
        z = depth * np.sin(altitude)
    n = x.shape[0]

    # 取得できないフィールドは0埋め（nuScenesは “TYPE I”=符号付き整数 で読む点に注意）
    dyn_prop = np.zeros(n, dtype=np.int8)     # SIZE=1, TYPE=I -> 'b'
    id_col   = np.arange(n, dtype=np.int16)   # SIZE=2, TYPE=I -> 'h'
    rcs      = np.zeros(n, dtype=np.float32)  # 'f'
    vx       = np.zeros(n, dtype=np.float32)
    vy       = np.zeros(n, dtype=np.float32)
    vx_comp  = np.zeros(n, dtype=np.float32)
    vy_comp  = np.zeros(n, dtype=np.float32)
    is_quality_valid = np.zeros(n, dtype=np.int8)
    ambig_state      = np.zeros(n, dtype=np.int8)
    x_rms            = np.zeros(n, dtype=np.int8)
    y_rms            = np.zeros(n, dtype=np.int8)
    invalid_state    = np.zeros(n, dtype=np.int8)
    pdh0             = np.zeros(n, dtype=np.int8)
    vx_rms           = np.zeros(n, dtype=np.int8)
    vy_rms           = np.zeros(n, dtype=np.int8)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z dyn_prop id rcs vx vy vx_comp vy_comp "
        "is_quality_valid ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms\n"
        "SIZE 4 4 4 1 2 4 4 4 4 4 1 1 1 1 1 1 1 1\n"
        "TYPE F F F I I F F F F F I I I I I I I I\n"
        "COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )

    packer = struct.Struct('<fff b h f f f f f b b b b b b b b')  # 合計 43 bytes/point

    with open(pcd_path, 'wb') as f:
        f.write(header.encode('ascii'))
        # 本体
        for i in range(n):
            f.write(packer.pack(
                float(x[i]), float(y[i]), float(z[i]),
                int(dyn_prop[i]), int(id_col[i]), float(rcs[i]),
                float(vx[i]), float(vy[i]), float(vx_comp[i]), float(vy_comp[i]),
                int(is_quality_valid[i]), int(ambig_state[i]),
                int(x_rms[i]), int(y_rms[i]), int(invalid_state[i]),
                int(pdh0[i]), int(vx_rms[i]), int(vy_rms[i])
            ))
        # ★ 末尾にダミー1バイト（改行）を追加：nuScenesの厳格チェック対策
        f.write(b'\n')





def batch_convert_radar(bin_root: str):
    """
    bin_root 配下の RADAR_* ディレクトリにある .bin を .pcd へ変換。
    出力は同じディレクトリ（拡張子だけ .pcd）。
    """
    for root, _, files in os.walk(bin_root):
        rel = os.path.relpath(root, bin_root)
        parts = [p for p in rel.split(os.sep) if p != '.']
        if not any(p.startswith("RADAR_") for p in parts):
            continue
        targets = [f for f in files if f.endswith(".bin")]
        for fname in tqdm(targets, desc=f"Converting {rel}"):
            bin_path = os.path.join(root, fname)
            pcd_path = os.path.join(root, fname[:-4] + ".pcd")
            convert_bin_to_nuscenes_pcd(bin_path, pcd_path)

if __name__ == "__main__":
    base = "./data/nuscenes_eval"
    for sub in ["sweeps", "samples"]:
        path = os.path.join(base, sub)
        if os.path.isdir(path):
            print(f"▶ RADAR_* in {sub}: .bin → .pcd")
            batch_convert_radar(path)
    print("✅ RADAR_* の .bin → .pcd 変換完了（nuScenes完全一致 + 末尾1byteパディング）")
