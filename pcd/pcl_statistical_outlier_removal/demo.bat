by lixin 202206
set PATH=D:\carlos\graduate_files\lixin\install\PCL 1.10.0\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\FLANN\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\VTK\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\Qhull\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\OpenNI2\Tools;%PATH%

cd ..\..\
.\pcl_statistical_outlier_removal.exe
.\pcl_statistical_outlier_removal.exe .\data\table_scene_lms400.pcd .\data\table_scene_lms400_sor_50_2.pcd 50 2
# before pcl_statistical_outlier_removal
.\open3d_visualizer.exe --path_source .\data\table_scene_lms400.pcd
# after pcl_statistical_outlier_removal
.\open3d_visualizer.exe --path_source .\data\table_scene_lms400_sor_50_2.pcd

pause