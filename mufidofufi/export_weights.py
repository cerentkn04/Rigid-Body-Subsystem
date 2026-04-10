"""
One-time weight export — uses h5py only (no TensorFlow needed).
Run after training to produce runic_weights.bin.

Usage: python export_weights.py
"""

import h5py
import numpy as np

BASE = 'model_weights'
LAYERS = [
    (f'{BASE}/conv2d/sequential/conv2d/kernel',   (3,3,1,32)),
    (f'{BASE}/conv2d/sequential/conv2d/bias',      (32,)),
    (f'{BASE}/conv2d_1/sequential/conv2d_1/kernel',(3,3,32,64)),
    (f'{BASE}/conv2d_1/sequential/conv2d_1/bias',  (64,)),
    (f'{BASE}/dense/sequential/dense/kernel',      (2304,128)),
    (f'{BASE}/dense/sequential/dense/bias',        (128,)),
    (f'{BASE}/dense_1/sequential/dense_1/kernel',  (128,6)),
    (f'{BASE}/dense_1/sequential/dense_1/bias',    (6,)),
]

with h5py.File('runic_model.h5', 'r') as f:
    with open('runic_weights.bin', 'wb') as out:
        for path, expected_shape in LAYERS:
            data = np.array(f[path], dtype=np.float32)
            assert data.shape == expected_shape, \
                f"{path}: expected {expected_shape}, got {data.shape}"
            out.write(data.tobytes())
            print(f"  {path.split('/')[-1]}: {data.shape}")

print("\nExported to runic_weights.bin")
import os
print(f"File size: {os.path.getsize('runic_weights.bin') / 1024:.1f} KB")
