#ifndef WALL_H
#define WALL_H
#include "includes.h"
#include "line.h"
#include "pcl_view.h"

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

    bool merge(Wall w){
        std::vector<PointPair> pPairResult, pPair1, pPair2;
        pPair1 = pointPairList;
        pPair2 = w.getPairList();

        for (int i = 0; i < pPair2.size(); i++){
            pPair2[i].pStart.y = l.getY(pPair2[i].pStart.x);
            pPair2[i].pEnd.y = l.getY(pPair2[i].pEnd.x);
        }

        PointPair currentPointPair, mergedPair;

        int iP1 = 0, iP2 = 0;
        mergedPair = ((pPair1[iP1].pStart.x < pPair2[iP2].pStart.x) || (pPair1[iP1].pStart.y < pPair2[iP2].pStart.y)) ?
                    pPair1[iP1++] : pPair2[iP2++];

        for ( ; iP1 < pPair1.size() && iP2 < pPair2.size(); ){
            currentPointPair = ((pPair1[iP1].pStart.x < pPair2[iP2].pStart.x) || (pPair1[iP1].pStart.y < pPair2[iP2].pStart.y)) ?
                        pPair1[iP1++] : pPair2[iP2++];
            if (currentPointPair.pStart.x < mergedPair.pEnd.x || currentPointPair.pStart.y < mergedPair.pEnd.y){
                if ((currentPointPair.pEnd.x > mergedPair.pEnd.x) || (currentPointPair.pEnd.y > mergedPair.pEnd.y)){
                    mergedPair.pEnd = currentPointPair.pEnd;
                }
                continue;
            }

            pPairResult.push_back(mergedPair);
            mergedPair = currentPointPair;
        }

        while (iP1 < pPair1.size()){
            pPairResult.push_back(pPair1[iP1++]);
        }
        while (iP2 < pPair2.size()){
            pPairResult.push_back(pPair2[iP2++]);
        }

        pointPairList = pPairResult;
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
        if (nUnknown >= nEmpty)
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
                if (distance(pEnd, pCurrent) <= precision * 10){
                    pEnd = pCurrent;
                    continue;
                }
                else {

                    pMiddle.x = (pEnd.x + pCurrent.x) / 2.0f;
                    pMiddle.y = l.getY(pMiddle.x);
                    if (isObstacle(pMiddle) && !isObstacle(pCurrent) && !isObstacle(pEnd)){
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

    float length(){
        float length = 0, dx, dy, d;
        int pairSize = pointPairList.size();
        for (int i = 0; i < pairSize; i++){
            dx = pointPairList[i].pStart.x - pointPairList[i].pEnd.x;
            dy = pointPairList[i].pStart.y - pointPairList[i].pEnd.y;
            d = sqrt(dx * dx + dy * dy);
            length += d;
        }
        return length;
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

    float getPrecision(){
        return precision;
    }

    Line getLine(){
        return l;
    }

private:
    float precision, resolution, originX, originY;
    int width, height, mapSize;
    nav_msgs::OccupancyGridConstPtr mapMsg;
    Line l;
    std::vector<PointPair> pointPairList;
    pcl::PointCloud<PointT>::Ptr cloud;

};

std::vector<Wall> wall_merge(std::vector<Wall> walls){


    int wallSize = walls.size();

    std::cout<< "Wall Merge Begin, wallSize: " << wallSize << endl;

    std::vector<Wall> wallPositive, wallNegative;
    for (int i = 0; i < wallSize; i++){
        Wall wallTmp = walls[i];
        if (fabs(wallTmp.getLine().a) < TOLERANCE || wallTmp.getLine().a > 0){
            wallPositive.push_back(wallTmp);
        }
        else {
            wallNegative.push_back(wallTmp);
        }
    }

    std::cout<< "Positive wall size: " << wallPositive.size()
             << ", Negative wall size: " << wallNegative.size() <<endl;

    float angleTolerance = 5 * M_PI / 180, distanceTolerance = 0.5;
    bool merged = false;

    do {

        merged = false;
        int positiveSize = wallPositive.size();

        std::cout<< "Current Positive wall size: " << positiveSize <<endl;

        for (int i = 0; i < positiveSize - 1; i++){
            for (int j = i + 1; j < positiveSize; j++){
                Line l1 = wallPositive[i].getLine(), l2 = wallPositive[j].getLine();
                if ((fabs(atan(l1.a) - atan(l2.a)) < angleTolerance)){
                    Wall w1 = wallPositive[i], w2 = wallPositive[j];
                    std::vector<PointPair> w2Pair = w2.getPairList();
                    Point pStart = w2Pair.front().pStart, pEnd = w2Pair.back().pEnd;
                    if (l1.pointDistance(pStart.x, pStart.y) < distanceTolerance && l1.pointDistance(pEnd.x, pEnd.y) < distanceTolerance){
                        if (w1.length() > w2.length()){
                            w1.merge(w2);
                            wallPositive[i] = w1;
                        }
                        else {
                            w2.merge(w1);
                            wallPositive[i] = w2;
                        }
                        wallPositive.erase(wallPositive.begin() + j);
                        merged = true;
                        break;
                    }
                }
            }
            if (merged)
                break;
        }
    } while (merged);

    do {

        merged = false;
        int negativeSize = wallNegative.size();

        std::cout<< "Current Negative wall size: " << negativeSize <<endl;

        for (int i = 0; i < negativeSize - 1; i++){
            for (int j = i + 1; j < negativeSize; j++){
                Line l1 = wallNegative[i].getLine(), l2 = wallNegative[j].getLine();
                if ((fabs(atan(l1.a) - atan(l2.a)) < angleTolerance)){
                    Wall w1 = wallNegative[i], w2 = wallNegative[j];
                    std::vector<PointPair> w2Pair = w2.getPairList();
                    Point pStart = w2Pair.front().pStart, pEnd = w2Pair.back().pEnd;
                    if (l1.pointDistance(pStart.x, pStart.y) < distanceTolerance && l1.pointDistance(pEnd.x, pEnd.y) < distanceTolerance){
                        if (w1.length() > w2.length()){
                            w1.merge(w2);
                            wallNegative[i] = w1;
                        }
                        else {
                            w2.merge(w1);
                            wallNegative[i] = w2;
                        }
                        wallNegative.erase(wallNegative.begin() + j);
                        merged = true;
                        break;
                    }
                }
            }
            if (merged)
                break;
        }
    } while (merged);

    return wallNegative;
}

void show_wall(std::vector<Wall> walls, double minLength = -1){
    int n = 0;
    srand(time(NULL));
    cout<< "Walls Size: " << walls.size() <<endl;

    //pcl::visualization::PCLVisualizer viewer("wall viewer");

    /*

    for (int i = 0; i < walls.size() / 10; i++){
        Wall wall = walls[i];
        //int r = rand() % 255, g = rand() % 255, b = rand() % 255;

        std::vector<PointPair> pointPairs = wall.getPairList();
        int r = (i * 17 + 27) % 256,
                g = (i * 27 + 37) % 256,
                b = (i * 37 + 47) % 256;
        cout<< "PairSize: " << pointPairs.size() <<endl;
        cout<< "Color: " << "(" << r << ", " << g << ", " << b << ")" <<endl;
        for (int j = 0; j < pointPairs.size(); j++){

            PointPair p = pointPairs[j];
            viewer.addLine<PointT>(PointT(p.pStart.x, p.pStart.y, 0), PointT(p.pEnd.x, p.pEnd.y, 0), r, g, b, boost::lexical_cast<std::string>(i * 77777 + j * 777 + 77), 0);
            n++;
        }
    }
    */

    if (minLength <= 0){
        minLength = 10 * walls[0].getPrecision();
    }

    std::vector<pcl::PointCloud<PointT>::Ptr > cloudVector;
    for (int i = 0; i < walls.size(); i++){
        Wall wall = walls[i];
        float wLength = wall.length();
        if (wLength <= minLength)
            continue;

        n++;
        int nInsert = int(wLength / wall.getPrecision());
        float insertStep = 1.0 * wLength / nInsert;

        std::vector<PointPair> pointPairs = wall.getPairList();
        pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>);
        pCloud->height = 1; pCloud->width = pointPairs.size() + nInsert;
        pCloud->resize(pCloud->height * pCloud->width);
        int cloudIndex = 0;
        for (int i = 0; i < pointPairs.size(); i++){
            float xStart = pointPairs[i].pStart.x,
                    yStart = pointPairs[i].pStart.y,
                    xEnd = pointPairs[i].pEnd.x,
                    yEnd = pointPairs[i].pEnd.y;
            float pLength = sqrt((xEnd - xStart) * (xEnd - xStart) + (yEnd - yStart) * (yEnd - yStart));
            int nCurrentInsert = int(pLength / insertStep);
            if (nCurrentInsert == 0){
                pCloud->points[cloudIndex].x = xStart;
                pCloud->points[cloudIndex].y = yStart;
                pCloud->points[cloudIndex].z = 0;
                cloudIndex++;
                continue;
            }
            float xStep = (xEnd - xStart) / nCurrentInsert, yStep = (yEnd - yStart) / nCurrentInsert;
            float x = xStart, y = yStart;
            for (int j = 0; j < nCurrentInsert; j++, cloudIndex++, x += xStep, y += yStep){
                pCloud->points[cloudIndex].x = x;
                pCloud->points[cloudIndex].y = y;
                pCloud->points[cloudIndex].z = 0;
            }
        }
        cloudVector.push_back(pCloud);
    }

    cout<< "Lines: " << n << endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVectorVis(cloudVector);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


}

#endif // WALL_H
