#include "wall.h"
#include "linear_segmentation.h"

std::vector<Wall> walls;
nav_msgs::OccupancyGridConstPtr mapMsg;
std::vector<Wall> showWalls;
float minLength = -1;
bool saved = false;
void Map_Callback(const nav_msgs::OccupancyGridConstPtr &map_msg){
    if (saved){
        return;
    }
    mapMsg = map_msg;
    saved = true;

    for (int i = 0; i < walls.size(); i++){
        Wall wall = walls[i];
        wall.setMapMsg(mapMsg);
        wall.sort();
        wall.extend();
        wall.print();
        showWalls.push_back(wall);
    }

    show_wall(showWalls, minLength);
    show_wall(wall_merge(showWalls), minLength);
}

int main(int argc, char * argv[]){

    bool debug = true, visual = true;

    float ftemp;
    if (pcl::console::parse(argc, argv, "-l", ftemp) >= 0){
        if (ftemp > 0){
            minLength = ftemp;
        }
    }

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    std::string pcd_file = argv[filenames[0]];
    pcl::PointCloud<PointT>::Ptr map_cloud (new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(pcd_file, *map_cloud);
    walls = Laser_Callback_Cloud(map_cloud, visual, debug);

    ros::init(argc, argv, "wall");

    ros::NodeHandle nMap;
    ros::Subscriber nmapSub = nMap.subscribe("/map", 1, Map_Callback);

    ros::spin();
}
