#ifndef WALL_H
#define WALL_H
#include "includes.h"
#include "line.h"

struct Point{
    float x, y;
};

struct PointPair{
    Point pStart, pEnd;
};

class Wall{

public:
    Wall(Line l, pcl::PointCloud<PointT>::Ptr cloud){
        this->l = l;
        this->cloud = cloud;
        precision = 0.025;
    }

    void setMapMsg(nav_msgs::OccupancyGridConstPtr &mapMsg){
        this->mapMsg = mapMsg;
        originX = mapMsg->info.origin.position.x;
        originY = mapMsg->info.origin.position.y;
        resolution = mapMsg->info.resolution;
        width = mapMsg->info.width;
        height = mapMsg->info.height;
        mapSize = width * height;
    }

    void sort(){
        int cloudSize = cloud->size();
        for (int i = 0; i < cloudSize - 1; i++){
            for (int j = i+1; j < cloudSize; j++){
                if ((cloud->points.at(i).x > cloud->points.at(j).x)
                        || (cloud->points.at(i).y > cloud->points.at(j).y)){
                    float xTemp = cloud->points[i].x, yTemp = cloud->points[i].y;
                    cloud->points[i].x = cloud->points[j].x;
                    cloud->points[i].y = cloud->points[j].y;
                    cloud->points[j].x = xTemp;
                    cloud->points[j].y = yTemp;
                }
            }
        }
    }

    float distance(Point p1, Point p2){
        float dx = p1.x - p2.x, dy = p1.y - p2.y;
        return sqrt(dx * dx + dy * dy);
    }

    bool isObstacle(Point p){

        /*
            int x = i % width;
            int y = (i - x) / width;
            //cout<< "(x, y): " << "(" << x << "," << y <<")" <<endl;
            map_cloud->points.at(nCount).x = x * resolution + originX;
            map_cloud->points.at(nCount).y = -y * resolution - originY;
         */

        int iX = (int)((p.x - originX) / resolution), iY = (int)((-originY - p.y) / resolution);
        cout<< "(iX: " << iX << ", iY: " << iY << "), ";
        int index = iY * width + iX;
        int nEmpty = 0, nObstacle = 0, nUnknown = 0;
        for (int i = -width; i <= width; i += width){
            for (int j = -1; j <= 1; j++){
                int currentIndex = index + j + i;
                if (currentIndex < 0 || currentIndex >= mapSize)
                    continue;
                switch (mapMsg->data[currentIndex]){
                case 0:
                    nEmpty++;
                    break;

                case -1:
                    nUnknown++;
                    break;

                case 100:
                    nObstacle++;
                    break;

                default:
                    nUnknown++;
                    break;
                }
            }
        }

        cout<< "nObstacle: " << nObstacle << ", nUnknown: " << nUnknown << ", nEmpty: " << nEmpty <<endl;
        if (nObstacle + nUnknown >= nEmpty)
            return true;
        return false;
    }

    void extend(){
        int cloudSize = cloud->size();

        if (l.a != 0 && fabs(l.a) < 1 / TOLERANCE){

            Point pStart, pEnd, pCurrent, pMiddle;
            pStart.x = cloud->points[0].x, pStart.y = l.getY(pStart.x);
            pEnd = pStart;
            for (int i = 1; i < cloudSize; i++){
                pCurrent.x = cloud->points[i].x; pCurrent.y = l.getY(pCurrent.x);
                if (distance(pEnd, pCurrent) <= precision * 3){
                    pEnd = pCurrent;
                    continue;
                }
                else {
                    pMiddle.x = (pEnd.x + pCurrent.x) / 2.0f;
                    pMiddle.y = l.getY(pMiddle.x);
                    if (isObstacle(pMiddle)){
                        pEnd = pCurrent;
                    }
                    else {
                        PointPair pPair;
                        pPair.pStart = pStart; pPair.pEnd = pEnd;
                        pointPairList.push_back(pPair);
                        pStart = pEnd = pCurrent;
                    }
                }
            }
            if (pStart.x != pEnd.x){
                PointPair pPair;
                pPair.pStart = pStart; pPair.pEnd = pEnd;
                pointPairList.push_back(pPair);
            }
        }
    }

    void print(){
        int listSize = pointPairList.size();

        if (listSize == 0){
            cout<< "Empty" <<endl;
            return;
        }

        cout<< "PairList size: " << listSize;
        for (int i = 0; i < listSize; i++){
            PointPair pPair = pointPairList[i];
            cout<< "{";
            cout<< "(" << pPair.pStart.x << ", " << pPair.pStart.y << "), "
                << "(" << pPair.pEnd.x << ", " << pPair.pEnd.y << ")"
                << "} ";
        }

        cout<< endl;
    }

    std::vector<PointPair> getPairList(){
        return pointPairList;
    }

private:
    float precision, resolution, originX, originY;
    int width, height, mapSize;
    nav_msgs::OccupancyGridConstPtr mapMsg;
    Line l;
    std::vector<PointPair> pointPairList;
    pcl::PointCloud<PointT>::Ptr cloud;

};


void show_wall(std::vector<Wall> walls){
    int n = 0;
    srand(time(NULL));
    cout<< "Walls Size: " << walls.size() <<endl;

    pcl::visualization::PCLVisualizer viewer("wall viewer");

    for (int i = 0; i < walls.size(); i++){
        Wall wall = walls[i];
        int r = rand() % 256, g = rand() % 256, b = rand() % 256;
        std::vector<PointPair> pointPairs = wall.getPairList();
        cout<< "PairSize: " << pointPairs.size() <<endl;
        for (int j = 0; j < pointPairs.size(); j++){

            PointPair p = pointPairs[j];
            viewer.addLine<PointT>(PointT(p.pStart.x, p.pStart.y, 0), PointT(p.pEnd.x, p.pEnd.y, 0), r, g, b, boost::lexical_cast<std::string>(i * walls.size() + j + rand()), 0);
            n++;
        }
    }

    cout<< "Lines: " << n << endl;

    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }

}

#endif // WALL_H
