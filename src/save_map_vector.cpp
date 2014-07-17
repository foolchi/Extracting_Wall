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
    vector<int> map(map_msg->data.begin(), map_msg->data.end());
    ofstream FILE(msg_vector_file.c_str(), ios::out | ofstream::binary);
    copy(map.begin(), map.end(), ostreambuf_iterator<char> (FILE));
    FILE.close();

    saved = true;
}

void save(){
    vector<int> map;
    for (int i = 0; i < 1000; i++)
        map.push_back(i);
    ofstream FILE(msg_vector_file.c_str(), ios::out | ofstream::binary);
    copy(map.begin(), map.end(), ostreambuf_iterator<char> (FILE));
    FILE.close();
    cout<<"Saved"<<endl;
}

void read(){
    vector<int> data;
    ifstream INFILE(msg_vector_file.c_str(), ios::in | ifstream::binary);

    if (INFILE){
        istream_iterator<int> iBegin(INFILE);
        copy(iBegin, istream_iterator<int>(), back_inserter(data));
    }
    else {
        cout<<"Cannot open the file"<<endl;
    }
    cout<<"Vector size: " << data.size() << endl;
    for (int i = 0; i < data.size(); i++){
        cout<<data.at(i)<<endl;
    }

}

int main(int argc, char *argv[]){
    //ros::init(argc, argv, "save_map_msg");
    //save();
    //read();
    ros::NodeHandle nMap;
    ros::Subscriber nmapSub = nMap.subscribe("/map", 1, Map_Callback);

    //ros::spin();

}

