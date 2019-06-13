#include "pclviewer.h"
#include "./build/ui_pclviewer.h"
#include <boost/make_shared.hpp>
#include <pcl/registration/transforms.h>
#include <QDebug>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};



PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer),
    filtering_axis_ (1),  // = y
    color_mode_ (4)  // = Rainbow
{
  ui->setupUi (this);
  this->setWindowTitle ("Deng DX PCL viewer");

  //设置份数滑动条的范围和步进
  ui->horizontalSlider->setRange(30, 1000);
  ui->horizontalSlider->setSingleStep(20);
  //设置滑动条的刻度显示和刻度间隔
  ui->horizontalSlider->setTickPosition(QSlider::TicksBothSides);
  ui->horizontalSlider->setTickInterval(20);

  //同步份数的计数器和滑动条
  connect(ui->spinBoxCount, SIGNAL(valueChanged(int)),
          ui->horizontalSlider, SLOT(setValue(int)));
  connect(ui->horizontalSlider, SIGNAL(valueChanged(int)),
          ui->spinBoxCount, SLOT(setValue(int)));

   cloud_left.reset(new PointCloudT);
   cloud_right.reset(new PointCloudT);
  // Setup the cloud pointer
  cloud_.reset (new PointCloudT);
  // The number of points in the cloud
  cloud_->resize (500);

  // Fill the cloud with random points
  for (size_t i = 0; i < cloud_->points.size (); ++i)
  {
    cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  // Set up the QVTK window
  viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer_->setBackgroundColor (0.1, 0.1, 0.1);
  ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "Load" and "Save" buttons and their functions
  connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
  connect (ui->pushButton_save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));

  //关于我自己添加的槽函数
  connect (ui->pushButton_open_left, SIGNAL(clicked ()), this, SLOT(open_left_FileButtonPressed ()));
  connect (ui->pushButton_open_right, SIGNAL(clicked ()), this, SLOT(open_right_FileButtonPressed ()));
  // Connect X,Y,Z radio buttons and their functions
  connect (ui->radioButton_x, SIGNAL(clicked ()), this, SLOT(axisChosen ()));
  connect (ui->radioButton_y, SIGNAL(clicked ()), this, SLOT(axisChosen ()));
  connect (ui->radioButton_z, SIGNAL(clicked ()), this, SLOT(axisChosen ()));

  connect (ui->radioButton_BlueRed, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_GreenMagenta, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_WhiteRed, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_GreyRed, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_Rainbow, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));

  // Color the randomly generated cloud
  colorCloudDistances ();
  viewer_->addPointCloud (cloud_, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}

void
PCLViewer::loadFileButtonPressed ()
{
    viewer_->removePointCloud("cloud_right");
    viewer_->removePointCloud("cloud_left");
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/salm/myopencv/path_planning/data/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }
  /*
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud_);
  sor.setLeafSize (0.05f,0.05f,0.05f);
  sor.filter (*cloud_);
*/
  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

void
PCLViewer::saveFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/salm/myopencv/yl_pcl/data/tutorials/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}

void
PCLViewer::axisChosen ()
{
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
  if (ui->radioButton_x->isChecked ())
  {
    PCL_INFO("x filtering chosen\n");
    filtering_axis_ = 0;
  }
  else if (ui->radioButton_y->isChecked ())
  {
    PCL_INFO("y filtering chosen\n");
    filtering_axis_ = 1;
  }
  else
  {
    PCL_INFO("z filtering chosen\n");
    filtering_axis_ = 2;
  }

  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::lookUpTableChosen ()
{
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
  if (ui->radioButton_BlueRed->isChecked ())
  {
    PCL_INFO("Blue -> Red LUT chosen\n");
    color_mode_ = 0;
  }
  else if (ui->radioButton_GreenMagenta->isChecked ())
  {
    PCL_INFO("Green -> Magenta LUT chosen\n");
    color_mode_ = 1;
  }
  else if (ui->radioButton_WhiteRed->isChecked ())
  {
    PCL_INFO("White -> Red LUT chosen\n");
    color_mode_ = 2;
  }
  else if (ui->radioButton_GreyRed->isChecked ())
  {
    PCL_INFO("Grey / Red LUT chosen\n");
    color_mode_ = 3;
  }
  else
  {
    PCL_INFO("Rainbow LUT chosen\n");
    color_mode_ = 4;
  }

  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::colorCloudDistances ()
{
  // Find the minimum and maximum values along the selected axis
  double min, max;
  // Set an initial value
  switch (filtering_axis_)
  {
    case 0:  // x
      min = cloud_->points[0].x;
      max = cloud_->points[0].x;
      break;
    case 1:  // y
      min = cloud_->points[0].y;
      max = cloud_->points[0].y;
      break;
    default:  // z
      min = cloud_->points[0].z;
      max = cloud_->points[0].z;
      break;
  }

  // Search for the minimum/maximum
  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    switch (filtering_axis_)
    {
      case 0:  // x
        if (min > cloud_it->x)
          min = cloud_it->x;

        if (max < cloud_it->x)
          max = cloud_it->x;
        break;
      case 1:  // y
        if (min > cloud_it->y)
          min = cloud_it->y;

        if (max < cloud_it->y)
          max = cloud_it->y;
        break;
      default:  // z
        if (min > cloud_it->z)
          min = cloud_it->z;

        if (max < cloud_it->z)
          max = cloud_it->z;
        break;
    }
  }

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    int value;
    switch (filtering_axis_)
    {
      case 0:  // x
        value = boost::math::iround ( (cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = boost::math::iround ( (cloud_it->y - min) * lut_scale);
        break;
      default:  // z
        value = boost::math::iround ( (cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_)
    {
      case 0:
        // Blue (= min) -> Red (= max)
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          cloud_it->r = 255;
          cloud_it->g = 0;
          cloud_it->b = 0;
        }
        else
        {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    }
  }
}



void
PCLViewer::open_left_FileButtonPressed ()
{
    viewer_->removePointCloud("cloud"); //
    viewer_->removePointCloud("cloud_right");
     viewer_->removePointCloud("cloud_left");
    std::vector <std::string> files;
    const std::string   extension=".pcd";
    //extension=".pcd";
    QString dir = QFileDialog::getExistingDirectory (0, "Please select a directory containing .pcd files.");
     if (dir.isEmpty ())
     {
       return (QApplication::quit ());
     }
     path_dir_ = dir.toStdString ();
     if (path_dir_ == "" || !boost::filesystem::exists (path_dir_))
     {
       std::cerr << "ERROR in pclviewer.cpp: Invalid path\n  '" << path_dir_<< "'\n";
      // return (false);
     }
     boost::filesystem::directory_iterator it_end;
       for (boost::filesystem::directory_iterator it (path_dir_); it != it_end; ++it)
       {
         if (!is_directory (it->status ()) &&
             boost::algorithm::to_upper_copy (boost::filesystem::extension (it->path ())) == boost::algorithm::to_upper_copy (extension))
         {
           files.push_back (it->path ().string ());
         }
       }
       if (files.size () < 1)
         {
           return;
         }
      PointCloudT::Ptr cloud_temp (new PointCloudT);
      PointCloudT::Ptr cloud_source (new PointCloudT);
      PointCloudT::Ptr cloud_targat (new PointCloudT);
Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

         for (unsigned int i=1; i<files.size (); ++i)
         {
             // Load the cloud.
             pcl::PCDReader reader;
             if (reader.read (files[i], *cloud_source) < 0)
             {
               std::cerr << "ERROR in pclviewer.cpp: Could not read the pcd file.\n";
             }
             if (reader.read (files[i-1], *cloud_targat) < 0)
             {
               std::cerr << "ERROR in pclviewer.cpp: Could not read the pcd file.\n";
             }


       if (cloud_source->is_dense ||cloud_targat->is_dense)
        { pcl::copyPointCloud (*cloud_targat, *cloud_targat);
         pcl::copyPointCloud (*cloud_source, *cloud_source);}
       else
       {
         PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
         std::vector<int> vec;
         pcl::removeNaNFromPointCloud (*cloud_targat, *cloud_targat, vec);
         pcl::removeNaNFromPointCloud (*cloud_source, *cloud_targat, vec);
       }
       //对点云渲染
       PointCloudColorHandlerCustom<PointT> cloud_tgt_h (cloud_source, 0, 255, 0);
   /*    PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_targat, 255, 0, 0);
       //可视化
       viewer_->addPointCloud (cloud_targat,cloud_src_h,"cloud");

       viewer_->resetCamera ();
       ui->qvtkWidget->update ();
      */
             PCLViewer::pairAlign(cloud_source, cloud_targat, cloud_temp, pairTransform, true);
             //transform current pair into the global transform
             pcl::transformPointCloud (*cloud_temp, *cloud_, GlobalTransform);
             //update the global transform
             GlobalTransform = GlobalTransform * pairTransform;
             viewer_->addPointCloud(cloud_,cloud_tgt_h,"cloud");

           //  viewer_->updatePointCloud (cloud_source, "cloud");
             viewer_->resetCamera ();
             ui->qvtkWidget->update ();
       }
}


void
PCLViewer::open_right_FileButtonPressed ()
{
    viewer_->removePointCloud("cloud"); //
    viewer_->removePointCloud("cloud_right");
     viewer_->removePointCloud("cloud_left");
    std::vector <std::string> files;
    const std::string   extension=".pcd";
    //extension=".pcd";

    QString dir = QFileDialog::getExistingDirectory (0, "Please select a directory containing .pcd files.");
     if (dir.isEmpty ())
     {
       return (QApplication::quit ());
     }
     path_dir_ = dir.toStdString ();
     if (path_dir_ == "" || !boost::filesystem::exists (path_dir_))
     {
       std::cerr << "ERROR in pclviewer.cpp: Invalid path\n  '" << path_dir_<< "'\n";
      // return (false);
     }
     boost::filesystem::directory_iterator it_end;
       for (boost::filesystem::directory_iterator it (path_dir_); it != it_end; ++it)
       {
         if (!is_directory (it->status ()) &&
             boost::algorithm::to_upper_copy (boost::filesystem::extension (it->path ())) == boost::algorithm::to_upper_copy (extension))
         {
           files.push_back (it->path ().string ());
         }
       }
       if (files.size () < 1)
         {
           return;
         }
      PointCloudT::Ptr cloud_tmp (new PointCloudT);
         for (unsigned int i=1; i<files.size (); ++i)
         {
             // Load the cloud.
             pcl::PCDReader reader;
             if (reader.read (files[0], *cloud_tmp) < 0)
             {
               std::cerr << "ERROR in pclviewer.cpp: Could not read the pcd file.\n";
               //return (false);
             }
         }

       if (cloud_tmp->is_dense)
         pcl::copyPointCloud (*cloud_tmp, *cloud_);
       else
       {
         PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
         std::vector<int> vec;
         pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
       }

       viewer_->addPointCloud (cloud_, "cloud");

       viewer_->resetCamera ();
       ui->qvtkWidget->update ();
}


/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */

void
PCLViewer::pairAlign (PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)

{
  //是否下采样
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloudT::Ptr src (new PointCloudT);
  PointCloudT::Ptr tgt (new PointCloudT);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;    //计算法线
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree); //搜索的方式
  norm_est.setKSearch (60);  //K=30

  norm_est.setInputCloud (src);   //输入点云
  norm_est.compute (*points_with_normals_src);   //计算输入点云的法线并保存在
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);     //输入点云
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);


  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);   //默认是1e-6  10的-6次方
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);
  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (10);  //默认设置是2
  for (int i = 0; i < ui->spinBoxCount->value(); ++i)  //默认设置是30
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);
    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);//
    reg.align (*reg_result);  //配准

        //accumulate transformation between each Iteration
                //计算配准之间的转化
    Ti = reg.getFinalTransformation () * Ti;
        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();
  }

  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);


 // PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
 // PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);

    PCL_INFO (" the registration is over.\n");


  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }

