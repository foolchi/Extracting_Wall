#ifndef PCL_VIEW_H
#define PCL_VIEW_H

#include "includes.h"

void show_pcd(const std::string pcd_file){

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    pcl::visualization::PCLVisualizer viewer("pcl viewer");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, 255, 0, 0);
    viewer.addPointCloud(cloud, color, "pcl");

    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVectorVis (std::vector<pcl::PointCloud<PointT>::Ptr > cloudVector)
{
    srand(time(NULL));
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Line viewer"));

    for (int i = 0; i < cloudVector.size(); i++){
        pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*(cloudVector.at(i)), *temp_cloud);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> line_color(temp_cloud, rand() % 256, rand() % 256, rand() % 256);
        viewer->addPointCloud(temp_cloud, line_color, boost::lexical_cast<std::string> (i));
    }

    viewer->initCameraParameters ();
    return (viewer);
}

#endif // PCL_VIEW_H
