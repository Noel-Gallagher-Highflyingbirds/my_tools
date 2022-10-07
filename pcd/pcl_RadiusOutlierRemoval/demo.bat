by lixin 202206
cd ..\..\
.\pcl_RadiusOutlierRemoval.exe
.\pcl_RadiusOutlierRemoval.exe .\data\table_scene_lms400.pcd .\data\table_scene_lms400_radiusfilter_r0_05_n100.pcd 0.05 100
# before RadiusOutlierRemoval
.\open3d_visualizer.exe --path_source .\data\table_scene_lms400.pcd
# after pcl_RadiusOutlierRemoval
.\open3d_visualizer.exe --path_source .\data\table_scene_lms400_radiusfilter_r0_05_n100.pcd

pause