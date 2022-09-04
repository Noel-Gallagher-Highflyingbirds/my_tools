cmake -DPCL_DIR=/usr/local/PCL110 -S ./ -B ./build
cmake --build ./build --config Release --parallel 4
cp ./build/pcl_convert_intensity ../../
