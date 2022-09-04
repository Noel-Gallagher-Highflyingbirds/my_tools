by lixin 202206
set PATH=D:\carlos\graduate_files\lixin\install\PCL 1.10.0\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\FLANN\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\VTK\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\Qhull\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\OpenNI2\Tools;%PATH%

cd ..\..\
.\pcl_RadiusOutlierRemoval.exe
.\pcl_RadiusOutlierRemoval.exe .\data\table_scene_lms400.pcd .\data\table_scene_lms400_radiusfilter_r0_05_n100.pcd 0.05 100
# before RadiusOutlierRemoval
.\open3d_visualizer.exe --path_source .\data\table_scene_lms400.pcd
# after pcl_RadiusOutlierRemoval
.\open3d_visualizer.exe --path_source .\data\table_scene_lms400_radiusfilter_r0_05_n100.pcd

pause