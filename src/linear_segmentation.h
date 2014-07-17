#ifndef LINEAR_SEGMENTATION_H
#define LINEAR_SEGMENTATION_H
#include "includes.h"
#include "line.h"
#include "pcl_view.h"
#include "wall.h"

Line angle_lslf_cloud(pcl::PointCloud<PointT>::Ptr cloud){
    // Least square line fitting

    int nPoints = cloud->size();
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    float x, y;

    for (int i = 0; i < nPoints; i++){
        x = cloud->at(i).x;
        y = cloud->at(i).y;
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    float a, b; // y = a * x + b;

    if (fabs(sumX2 - sumX * sumX / nPoints) < TOLERANCE)
        a = (sumXY - sumX * sumY / nPoints) > 0 ? 1 / TOLERANCE : -1 / TOLERANCE;
    else
        a = (sumXY - sumX * sumY / nPoints) / (sumX2 - sumX * sumX / nPoints);

    b = (sumY - sumX * a) / nPoints;

    return Line(a, b);
}

/*
Line2 angle_lslf_cloud2(pcl::PointCloud<PointT>::Ptr cloud){
    // Least square line fitting

    int nPoints = cloud->size();
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    float x, y;

    for (int i = 0; i < nPoints; i++){
        x = cloud->at(i).x;
        y = cloud->at(i).y;
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    float a, b; // y = a * x + b;
    if (fabs(sumX2 - sumX * sumX / nPoints) < TOLERANCE){
        b = 0;
        a = nPoints / sumX;
        return Line2(a, b);
    }

    a = (sumXY - sumX * sumY / nPoints) / (sumX2 - sumX * sumX / nPoints);
    b = (sumY - sumX * a) / nPoints;

    if (fabs(b) < TOLERANCE){
        return
    }
    return Line(a, b);
}
*/

std::vector<Wall> Laser_Callback_Cloud(pcl::PointCloud<PointT>::Ptr laser_cloud, bool visual, bool debug){

    if (debug)
        std::cout<<"Laser callback cloud Begin"<<std::endl;

    std::vector<pcl::PointCloud<PointT>::Ptr > lineVector;

    //==============================================================
    // RANSAC algorithm to find the lines
    //==============================================================
    do {
        pcl::SampleConsensusModelLine<PointT>::Ptr
                model_l(new pcl::SampleConsensusModelLine<PointT> (laser_cloud));

        const boost::shared_ptr<std::vector<int> > inliers (new std::vector<int>);
        pcl::RandomSampleConsensus<PointT> ransac (model_l);
        ransac.setDistanceThreshold (.075);
        ransac.computeModel();
        ransac.getInliers(*inliers);

        if (debug)
            std::cout<<"Laser callback cloud Begin1"<<std::endl;

        if (laser_cloud->size() >= MIN_LINE_SIZE){

            if (debug)
                std::cout<<"Laser callback cloud Begin2"<<std::endl;

            pcl::PointCloud<PointT>::Ptr final(new pcl::PointCloud<PointT>);

            if (debug)
                std::cout<<"Laser callback cloud Begin3"<<std::endl;
            pcl::copyPointCloud<PointT>(*laser_cloud, *inliers, *final);

            if (debug)
                std::cout<<"Laser callback cloud Begin4"<<std::endl;
            if (final->points.size() >= MIN_LINE_SIZE)
                lineVector.push_back(final);
            std::cout<<"Cloud size before: "<<laser_cloud->size()<<std::endl;
            std::cout<<"Indice size: "<<inliers->size()<<std::endl;
            pcl::ExtractIndices<PointT> eifilter (true);
            eifilter.setInputCloud(laser_cloud);
            eifilter.setNegative(true);
            eifilter.setIndices(inliers);
            pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
            eifilter.filter(*cloud_out);
            pcl::copyPointCloud(*cloud_out, *laser_cloud);
            std::cout<<"Cloud size after: "<<laser_cloud->size()<<std::endl;
        }
        else
            break;

    }while (laser_cloud->size() >= MIN_LINE_SIZE);

    int lineSize = lineVector.size();
    std::cout<<"Vector size: "<<lineSize<<std::endl;
    for (int i = 0; i < lineSize; i++)
        std::cout<<"Line size: "<<lineVector.at(i)->size()<<std::endl;

    std::vector<Wall> walls;

    Line leftLine, rightLine;
    int leftMax = 0, rightMax = 0, leftMaxIndex = 0, rightMaxIndex = 0;
    for (int i = 0; i < lineVector.size(); i++){
        Line l = angle_lslf_cloud(lineVector.at(i));
        walls.push_back(Wall(l, lineVector.at(i)));
        std::cout.precision(3);
        std::cout<<"Line: y = "<<l.a<<" * x + " <<l.b<<" Angle: "<<atan(l.a) * 180 / M_PI<<" dOrigin: " <<fabs(l.b) / sqrt(l.a * l.a + 1) <<std::endl;
        if (l.a > 0 && lineVector.at(i)->size() > leftMax){
            leftLine = l;
            leftMaxIndex = i;
            leftMax = lineVector.at(i)->size();
        }
        else if (l.a < 0 && lineVector.at(i)->size() > rightMax){
            rightLine = l;
            rightMaxIndex = i;
            rightMax = lineVector.at(i)->size();
        }
    }

    //==============================================================
    // Visulization for lines
    //==============================================================
    if (visual){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = simpleVectorVis(lineVector);

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
    return walls;
}

#endif // LINEAR_SEGMENTATION_H
