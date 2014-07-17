#include "includes.h"
#include "linear_segmentation.h"

int main(int argc, char** argv){

    bool debug = true, visual = true;

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    std::string pcd_file = argv[filenames[0]];
    pcl::PointCloud<PointT>::Ptr map_cloud (new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(pcd_file, *map_cloud);
    Laser_Callback_Cloud(map_cloud, visual, debug);
}
