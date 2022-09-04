cmake -DPCL_ROOT=/usr/local/PCL110 -S ./ -B ./build
cmake --build ./build --config Release --parallel 4
cp ./build/pcl_statistical_outlier_removal ../../
