#include "includes.h"

int main(){
    pcl::visualization::PCLVisualizer viewer("wall viewer");

    viewer.addLine<PointT>(PointT(0, 0, 0), PointT(100, 100, 100), 255, 0, 0 ,"testline", 0);

    while(!viewer.wasStopped()){
        viewer.spin();
    }
}
