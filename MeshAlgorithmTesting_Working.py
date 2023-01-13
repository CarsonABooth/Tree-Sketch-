import open3d as o3d
import numpy as np
import os
import sys

#Import a single tree point cloud
pcd = o3d.io.read_point_cloud(
    "C:/Users/booth/Open3D/Data/DATA_clouds_ply/wytham_winter_5a.ply"
    )

#Estimate the normals for that point cloud
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30)
        )
pcd.orient_normals_consistent_tangent_plane(k=15)

#Reconstruct a mash using Poisson surface reconstruction

with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)
print(mesh)

#Remove low density verticies to increase tree structure definition 

vtr = densities < np.quantile(densities, 0.2)
mesh.remove_vertices_by_mask(vtr)

o3d.visualization.draw_geometries([mesh])

o3d.io.write_triangle_mesh("TestMesh1.ply", mesh)