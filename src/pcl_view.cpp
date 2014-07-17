#include "includes.h"

typedef pcl::PointXYZ PointT;

std::string pcd_file;

void Print_Help(){
    std::cerr<<"Usage:\tprogram\tpcd_file"<<std::endl;
}

void show_pcd(){

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    pcl::visualization::PCLVisualizer viewer("pcl viewer");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud, 255, 0, 0);
    viewer.addPointCloud(cloud, color, "pcl");

    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }

}

int main(int argc, char** argv){

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size () != 1)
        Print_Help();

    pcd_file = argv[filenames[0]];
    show_pcd();
}

