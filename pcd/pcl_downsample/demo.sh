by lixin 202207
../../pcl_convert_intensity --path_source ../../data/bun000.ply --path_target ../../data/bun000.pcd --format 1 --scale 1.0
./build/pcl_downsample ../../data/bun000.pcd ../../data/bun000_005.pcd 0 0.005
