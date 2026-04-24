#!/usr/bin/env python3
import trimesh
import os
import numpy as np

def mirror_stl_trimesh(input_file, output_file):
    """Mirror STL file along X axis using trimesh"""
    
    # Load the mesh
    mesh = trimesh.load(input_file)
    
    # Create mirror transformation matrix (negate X)
    mirror_matrix = np.array([
        [-1,  0,  0,  0],
        [ 0,  1,  0,  0],
        [ 0,  0,  1,  0],
        [ 0,  0,  0,  1]
    ])
    
    # Apply transformation
    mesh.apply_transform(mirror_matrix)
    
    # Save the mirrored mesh
    mesh.export(output_file)
    print(f"✓ Mirrored: {os.path.basename(input_file)} -> {os.path.basename(output_file)}")

# Process all wing files
meshes_dir = '/home/ros/rebird_ws/src/rebird_description/meshes'
wing_files = [
    'wing_t-1.stl',
    'wing_t-2.stl',
    'wing_t-3.stl',
    'wing_t-4.stl',
    'wing_t-5.stl'
]

print("Starting STL mirroring with trimesh...")
print("-" * 50)

for wing_file in wing_files:
    input_path = os.path.join(meshes_dir, wing_file)
    output_filename = wing_file.replace('.stl', '_left.stl')
    output_path = os.path.join(meshes_dir, output_filename)
    
    if os.path.exists(input_path):
        try:
            mirror_stl_trimesh(input_path, output_path)
        except Exception as e:
            print(f"✗ Error processing {wing_file}: {e}")
    else:
        print(f"✗ File not found: {input_path}")

print("-" * 50)
print("Done! Created mirrored STL files for left wing.")