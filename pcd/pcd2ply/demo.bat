by lixin 202206
set PATH=I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\bin;I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\3rdParty\FLANN\bin;I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\3rdParty\VTK\bin;I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\3rdParty\Qhull\bin;I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\3rdParty\OpenNI2\Tools;%PATH%

cd ..\..\
.\pcd2ply.exe -h
.\pcd2ply.exe .\data\maoxian_origin.pcd .\data\maoxian_origin2ply.ply

pause