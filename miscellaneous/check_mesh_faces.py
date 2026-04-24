import trimesh

path = "/home/ros/astr_rl_gym/resources/robots/astr_description/meshes/CB_Base_link.STL"
m = trimesh.load(path, force='mesh')

print(type(m))
print("Faces:", len(m.faces))
print("Vertices:", len(m.vertices))
print("Bounds:", m.bounds)
