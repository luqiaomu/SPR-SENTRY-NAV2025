import open3d as o3d
import numpy as np

print("->正在加载点云... ")
pcd = o3d.io.read_point_cloud("/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/result.pcd")
print(pcd)

# print("->正在可视化原始点云")
# o3d.visualization.draw_geometries([pcd])

print("->正在体素下采样...")
voxel_size = 0.08
downpcd = pcd.voxel_down_sample(voxel_size)
print(downpcd)

print("->正在可视化下采样点云")
o3d.visualization.draw_geometries([downpcd])
