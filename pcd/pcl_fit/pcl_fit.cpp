#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   viewer->addCoordinateSystem (1.0, "global");
  //viewer->initCameraParameters();
  return (viewer);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
create_line(double x0,
              double y0,
              double z0,
              double a,
              double b,
              double c,
              double point_size = 1000,
              double step = 0.1)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_line->width = point_size;
  cloud_line->height = 1;
  cloud_line->resize(cloud_line->width * cloud_line->height);

  for (std::size_t i = 0; i < cloud_line->points.size(); ++i) {
    cloud_line->points[i].x =
        x0 + a / std::pow(a * a + b * b + c * c, 0.5) * i * 0.1;
    cloud_line->points[i].y =
        y0 + b / std::pow(a * a + b * b + c * c, 0.5) * i * 0.1;
    cloud_line->points[i].z =
        z0 + c / std::pow(a * a + b * b + c * c, 0.5) * i * 0.1;
  }
  return cloud_line;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
create_plane(double a,
    double b,
    double c,
    double d,
    double point_size = 1000)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = point_size;
    cloud->height = 1;
    cloud->resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        if (c != 0) {
            cloud->points[i].x = 50 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 50 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].z = (-a * cloud->points[i].x - b * cloud->points[i].y - d) / c;
        }
        else {
            cloud->points[i].x = 50 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = -a * cloud->points[i].x - d;
            cloud->points[i].z = 50 * rand() / (RAND_MAX + 1.0);
        }
    }
    return cloud;
}


/**
  * \brief cretae a noise point cloud
  * \param pcd_origin the pcd you want to add noise
  * \param[in] size_pcd the number for the noise points
  * \param[in] range_min the min of the range
  * \param[in] range_max the max of the range
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr
create_noise_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_origin, std::size_t size_pcd,double range_min,double range_max) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise(new pcl::PointCloud<pcl::PointXYZ>);
    std::size_t noise_points_size = size_pcd;
    cloud_noise->width = noise_points_size;
    cloud_noise->height = 1;
    cloud_noise->points.resize(cloud_noise->width * cloud_noise->height);
    double range = range_max - range_min;
    // add noise
    for (std::size_t i = 0; i < noise_points_size; ++i) {
        int random_num = pcd_origin->points.size() * rand() / (RAND_MAX + 1.0f);
        cloud_noise->points[i].x =
            pcd_origin->points[random_num].x + range * rand() / (RAND_MAX + 1.0f) + range_min;
        cloud_noise->points[i].y =
            pcd_origin->points[random_num].y + range * rand() / (RAND_MAX + 1.0f) + range_min;
        cloud_noise->points[i].z =
            pcd_origin->points[random_num].z + range * rand() / (RAND_MAX + 1.0f) + range_min;
    }
    return cloud_noise;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr create_sphere(double x0, double y0, double z0, double r, std::size_t size_pcd = 1000) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. Generate cloud data
    int sphere_data_size = size_pcd;
    cloud->width = sphere_data_size;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // 1.2 Add sphere:
    double rand_x1 = 1;
    double rand_x2 = 1;
    for (std::size_t i = 0; i < sphere_data_size; ++i)
    {
        // See: http://mathworld.wolfram.com/SpherePointPicking.html
        while (pow(rand_x1, 2) + pow(rand_x2, 2) >= 1)
        {
            rand_x1 = (rand() % 100) / (50.0f) - 1;
            rand_x2 = (rand() % 100) / (50.0f) - 1;
        }
        double pre_calc = sqrt(1 - pow(rand_x1, 2) - pow(rand_x2, 2));
        cloud->points[i].x = 2 * rand_x1 * pre_calc * r + x0;
        cloud->points[i].y = 2 * rand_x2 * pre_calc * r + y0;
        cloud->points[i].z = (1 - 2 * (pow(rand_x1, 2) + pow(rand_x2, 2))) * r + z0;
        rand_x1 = 1;
        rand_x2 = 1;
    }
    return cloud;
}


pcl::ModelCoefficients::Ptr
fit_model(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::SacModel model_fit, double distance_threshold, bool vis=true)
{
    // fit model from a point cloud
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(model_fit);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (vis) {
        // extract segmentation part
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_model);

        // extract remain pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setNegative(true);
        extract.filter(*cloud_remain);

        //display origin pcd
        pcl::visualization::PCLVisualizer::Ptr viewer_ori;
        viewer_ori = simpleVis(cloud);
        while (!viewer_ori->wasStopped()) {
            viewer_ori->spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }

        pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);

        viewer->addPointCloud<pcl::PointXYZ>(cloud_remain, "cloud_remain");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_remain");

        viewer->addPointCloud<pcl::PointXYZ>(cloud_model, "cloud_model");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_model");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.5, "cloud_model");
        
        viewer->addCoordinateSystem(10.0, "global");

        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }
    }
    return coefficients;

}


void
demo_line()
{
    std::cout << "demo fit line" << std::endl;
  // line parameters
  double x0 = -2, y0 = -2, z0 = 0, a = 1, b = 1, c = 0;
  std::cout << "origin model parameters:"
      << "   (x - " << x0 << ") / " << a << " = (y - " << y0 << ") / " << b
      << " = (z - " << z0 << ") / " << c << std::endl;
  auto pcd_line_create = create_line(x0, y0, z0, a, b, c);

  pcl::PointCloud<pcl::PointXYZ>::Ptr line_with_noise(
      new pcl::PointCloud<pcl::PointXYZ>);
  //add noise
  auto cloud_noise = create_noise_cloud(pcd_line_create, pcd_line_create->points.size() / 4, -3, 3);
  *line_with_noise = *cloud_noise + *pcd_line_create;
  //pcl::io::savePLYFileBinary("./line_with_noise.ply", *line_with_noise);

  //fit_line(line_with_noise, 0.1);
  auto coefficients = fit_model(line_with_noise, pcl::SACMODEL_LINE, 0.1, true);
  // line parameters

  x0 = coefficients->values[0];
  y0 = coefficients->values[1];
  z0 = coefficients->values[2];
  a = coefficients->values[3];
  b = coefficients->values[4];
  c = coefficients->values[5];
  std::cout << "fit model parameters:"
      << "   (x - " << x0 << ") / " << a << " = (y - " << y0 << ") / " << b
      << " = (z - " << z0 << ") / " << c << std::endl;

}
void demo_sphere() {
    std::cout << "demo fit sphere" << std::endl;
    double x0 = 0, y0 = 0, z0 = 0, r = 4;
    std::cout << "origin model parameters:"
        << " x0: " << x0
        << " y0: " << y0
        << " z0: " << z0
        << " r: " << r << std::endl;
    auto pcd_sphere_create = create_sphere(x0, y0, z0, r, 10000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_with_noise(
        new pcl::PointCloud<pcl::PointXYZ>);
    auto cloud_noise = create_noise_cloud(pcd_sphere_create, pcd_sphere_create->points.size() / 10, -2, 2);

    *sphere_with_noise = *cloud_noise + *pcd_sphere_create;
    //pcl::io::savePLYFileBinary("./sphere_with_noise.ply", *sphere_with_noise);
    auto coefficients = fit_model(sphere_with_noise, pcl::SACMODEL_SPHERE, 0.01, true);
    // sphere parameters;

    x0 = coefficients->values[0];
    y0 = coefficients->values[1];
    z0 = coefficients->values[2];
    r = coefficients->values[3];
    std::cout << "fit model parameters:"
        << " x0: " << x0
        << " y0: " << y0
        << " z0: " << z0
        << " r: " << r << std::endl;

}

void demo_plane() {
    std::cout << "demo fit plane" << std::endl;
    double a = 1, b = 1, c = 0, d = 0;
    std::cout << "origin model parameters:"
        << " a: " << a
        << " b: " << b
        << " c: " << c
        << " d: " << d << std::endl;
    auto pcd_create = create_plane(a, b, c, d, 10000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_with_noise(
        new pcl::PointCloud<pcl::PointXYZ>);
    auto cloud_noise = create_noise_cloud(pcd_create, pcd_create->points.size() / 10, -2, 2);

    *sphere_with_noise = *cloud_noise + *pcd_create;
    //pcl::io::savePLYFileBinary("./plane_with_noise.ply", *sphere_with_noise);
    auto coefficients = fit_model(sphere_with_noise, pcl::SACMODEL_PLANE, 0.01, true);
    // sphere parameters;

    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];
    std::cout << "fit model parameters:"
        << " a: " << a
        << " b: " << b
        << " c: " << c
        << " d: " << d << std::endl;

}


int
main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cout << "please input parametars:\nfilepath\nfit_modle(1 line, 2 sphere, 3 plane)\ndistance_threshold\nvis(optional, 0:false 1:true)" << std::endl;
    demo_sphere();
    demo_line();
    demo_plane();
    return -1;
  }
  std::string file_path = argv[1];
  double model_need_fit = atoi(argv[2]);
  double distance_threshold = atof(argv[3]);
  double vis = atoi(argv[4]);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPLYFile(file_path, *cloud) < 0) {
    std::cout << "can not read file " << file_path << std::endl;
    return -1;
  }
  std::cout << "point size: " << cloud->points.size() << std::endl;
  if (model_need_fit == 1) {
      auto coefficients = fit_model(cloud, pcl::SACMODEL_LINE, distance_threshold, vis);
      // line parameters
      double x0, y0, z0, a, b, c;

      x0 = coefficients->values[0];
      y0 = coefficients->values[1];
      z0 = coefficients->values[2];
      a = coefficients->values[3];
      b = coefficients->values[4];
      c = coefficients->values[5];
      std::cout << "model parameters:"
          << "   (x - " << x0 << ") / " << a << " = (y - " << y0 << ") / " << b
          << " = (z - " << z0 << ") / " << c << std::endl;

  }
  if (model_need_fit == 2) {
      auto coefficients = fit_model(cloud, pcl::SACMODEL_SPHERE, distance_threshold, vis);
      // sphere parameters;
      double x0, y0, z0, r;

      x0 = coefficients->values[0];
      y0 = coefficients->values[1];
      z0 = coefficients->values[2];
      r = coefficients->values[3];
      std::cout << "model parameters:"
          << " x0: " << x0
          << " y0: " << y0
          << " z0: " << z0
          << " r: " << r << std::endl;
  }

  if (model_need_fit == 3) {
      auto coefficients = fit_model(cloud, pcl::SACMODEL_PLANE, distance_threshold, vis);
      //// line parameters: https://blog.csdn.net/qq_41102371/article/details/121482108
      double a, b, c, d;

      a = coefficients->values[0];
      b = coefficients->values[1];
      c = coefficients->values[2];
      d = coefficients->values[3];
      std::cout << "model parameters:"
          << " a: " << a
          << " b: " << b
          << " c: " << c
          << " d: " << d << std::endl;
  }
   

  return 0;
}


