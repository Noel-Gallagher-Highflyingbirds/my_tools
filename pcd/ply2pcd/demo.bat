by lixin 202206
set PATH=D:\carlos\graduate_files\lixin\install\PCL 1.10.0\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\FLANN\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\VTK\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\Qhull\bin;D:\carlos\graduate_files\lixin\install\PCL 1.10.0\3rdParty\OpenNI2\Tools;%PATH%

cd ..\..\bin
.\ply2pcd.exe
.\ply2pcd.exe ..\data\bun000.ply ..\data\bun000_2pcd.pcd

pause