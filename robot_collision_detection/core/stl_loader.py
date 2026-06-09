import struct
import numpy as np


def load_stl(filepath):
    """Load a binary STL file and return vertices and faces.

    Returns:
        vertices: (N, 3) array of unique vertices
        faces: (M, 3) array of face indices into vertices
    """
    with open(filepath, 'rb') as f:
        f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]

        all_verts = np.empty((num_triangles * 3, 3), dtype=np.float64)

        for i in range(num_triangles):
            f.read(12)  # normal
            for j in range(3):
                all_verts[i * 3 + j] = struct.unpack('<fff', f.read(12))
            f.read(2)  # attribute

    # Deduplicate vertices
    unique_verts, inverse = np.unique(
        np.round(all_verts, decimals=8), axis=0, return_inverse=True
    )
    faces = inverse.reshape(-1, 3)

    return unique_verts, faces
