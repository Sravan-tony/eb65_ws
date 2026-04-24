import open3d as o3d

src = "/home/ros/eb65_ws/src/eb65_description/meshes/036.stl"
dst = "/home/ros/eb65_ws/src/eb65_description/meshes/036_simplified.stl"

mesh = o3d.io.read_triangle_mesh(src)
print("Original triangles:", len(mesh.triangles))

mesh = mesh.simplify_quadric_decimation(50000)

mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()
mesh.compute_vertex_normals()
mesh.compute_triangle_normals()

print("New triangles:", len(mesh.triangles))

o3d.io.write_triangle_mesh(dst, mesh, write_ascii=False)
print("Saved simplified mesh to:", dst)