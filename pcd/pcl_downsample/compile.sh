cmake -DPCL_DIR=/usr/local/PCL110 -S ./ -B ./build
cmake --build ./build --config Release --parallel 8
cp ./build/pcl_downsample ../../bin
