#include "includes.h"

using namespace std;
bool saved = false;

void Map_Callback(const nav_msgs::OccupancyGridConstPtr &map_msg){
    if (saved){
        cout<<"saved!"<<endl;
        return;
    }
    /*
    Header header
        uint32 seq
        time stamp
        string frame_id
    MapMetaData info
        time map_load_time
        float32 resolution
        uint32 width
        uint32 height
        geometry_msgs/Pose origin
            geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
            geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
    int8[] data
*/
    int width = map_msg->info.width, height = map_msg->info.height;
    vector<int> data(map_msg->data.begin(), map_msg->data.end());
    cout<<"Datasize: " << map_msg->data.size()<<endl;
    cout<<"Width: "<<width<<", Height: "<<height<<endl;
    cout<<"Resolution: "<< map_msg->info.resolution<<endl;
    cout<< "("<<map_msg->info.origin.position.x
        << ", "<<map_msg->info.origin.position.y
        << ", "<<map_msg->info.origin.position.z
        << ")"<<endl;

    int size = map_msg->data.size();
    int nPoints = 0;
    for (size_t i = 0; i < size; i++){
        if (map_msg->data.at(i) == 0){
            //cout<<"Map["<<i<<"]"<<int(map_msg->data.at(i))<<endl;
            nPoints++;
        }
    }
    cout<< "nPoints: "<< nPoints <<endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    map_cloud->height = 1;
    map_cloud->width = nPoints;
    map_cloud->resize(map_cloud->height * map_cloud->width);
    float resolution = map_msg->info.resolution;
    float originX = map_msg->info.origin.position.x, originY = map_msg->info.origin.position.y;

    int nCount = 0;
    for (int i = 0; i < size; i++){
        if (map_msg->data.at(i) == 0){
            int y = i % width;
            int x = (i - y) / width;
            //cout<< "(x, y): " << "(" << x << "," << y <<")" <<endl;
            map_cloud->points.at(nCount).x = x * resolution + originX;
            map_cloud->points.at(nCount).y = y * resolution - originY;
            map_cloud->points.at(nCount).z = 0;
            nCount++;
            if (nCount >= nPoints)
                break;
        }
    }

    pcl::io::savePCDFile("map_msg.pcd", *map_cloud);
    saved = true;
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "save_map_msg");

    ros::NodeHandle nMap;
    ros::Subscriber nmapSub = nMap.subscribe("/map", 1, Map_Callback);

    ros::spin();
}

