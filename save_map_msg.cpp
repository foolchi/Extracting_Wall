#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

using namespace std;

void Map_Callback(const nav_msgs::OccupancyGridConstPtr &map_msg){
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
    vector<int> data = map_msg->data;
    cout<<"Datasize: " << data.size()<<endl;
    cout<<"Width: "<<width<<", Height: "<<height<<endl;
    cout<< "("<<map_msg->info.origin.position.x
        << ", "<<map_msg->info.origin.position.y
        << ", "<<map_msg->info.origin.position.z
        << ")"<<endl;
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "save_map_msg");

    ros::NodeHandle nMap;
    ros::Subscriber nmapSub = nMap.subscribe("/map", 1, Map_Callback);

    ros::spin();
}

