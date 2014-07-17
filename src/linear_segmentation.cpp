#include "includes.h"

bool debug = true, visual = true;

class CloudView{

public:
    CloudView(){
        //cloud = new pcl::PointCloud<pcl::PointXYZ>;
        flag = 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    int flag; // 1: leftWall, 2: floor, 3: rightWall, 4: line
};

std::vector<CloudView> cloudViewVector;

class Line{
    // Line : y = a * x + b;
public:
    Line(){
        a = b = 0;
    }

    Line(float a, float b){
        this->a = a; this->b = b;
    }
    Line(const Line & line){
        a = line.a; b = line.b;
    }

    float a, b;
};

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVectorVis (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloudVector)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Line viewer"));

    for (int i = 0; i < cloudVector.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(cloudVector.at(i)), *temp_cloud);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> line_color(temp_cloud, (100 + 50 * i) % 255, (100 + 100 * i) % 255, (100 + 150 * i) % 255);
        viewer->addPointCloud(temp_cloud, line_color, boost::lexical_cast<std::string> (i));
    }



    viewer->initCameraParameters ();
    return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer>
cloudVectorVis ()
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Final viewer"));

    std::cout<<"Cloud view size: "<<cloudViewVector.size()<<std::endl;

    for (int i = 0; i < cloudViewVector.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(cloudViewVector.at(i).cloud), *temp_cloud);
        std::cout<<"cloudViewVector["<<i<<"].size() = "<< temp_cloud->size()<<std::endl;


            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> default_color(temp_cloud, 100, 100, 100);
            viewer->addPointCloud(temp_cloud, default_color, boost::lexical_cast<std::string> (i));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr x_axis(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr y_axis(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr z_axis(new pcl::PointCloud<pcl::PointXYZ>);

    x_axis->width = 300; x_axis->height = 1; y_axis->width = 300; y_axis->height = 1; z_axis->width = 300; z_axis->height = 1;
    x_axis->resize(x_axis->height * x_axis->width);
    y_axis->resize(y_axis->height * y_axis->width);
    z_axis->resize(z_axis->height * z_axis->width);

    for (int i = 0; i < 300; i++){
        float value = 1.0 * i / 100;
        x_axis->points[i].x = value;
        x_axis->points[i].y = x_axis->points[i].z = 0;

        y_axis->points[i].y = value;
        y_axis->points[i].x = y_axis->points[i].z = 0;

        z_axis->points[i].z = value;
        z_axis->points[i].y = z_axis->points[i].x = 0;
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> x_color(x_axis, 255, 105, 180);
    viewer->addPointCloud(x_axis, x_color, "x_axis");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> y_color(y_axis, 208, 32, 144);
    viewer->addPointCloud(y_axis, y_color, "y_axis");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> z_color(z_axis, 255, 255, 255);
    viewer->addPointCloud(z_axis, z_color, "z_axis");

    viewer->initCameraParameters ();
    return (viewer);
}

Line angle_lslf_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud){
    // Least square line fitting

    int nPoints = laser_cloud->size();
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    float x, y;

    for (int i = 0; i < nPoints; i++){
        x = laser_cloud->at(i).x;
        y = laser_cloud->at(i).y;
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    float a, b; // y = a * x + b;
    a = (sumXY - sumX * sumY / nPoints) / (sumX2 - sumX * sumX / nPoints);
    b = (sumY - sumX * a) / nPoints;
    return Line(a, b);
}


void Laser_Callback_Cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud){

    if (debug)
        std::cout<<"Laser callback cloud Begin"<<std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > lineVector;

    //==============================================================
    // RANSAC algorithm to find the lines
    //==============================================================
    do {
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
                model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (laser_cloud));

        const boost::shared_ptr<std::vector<int> > inliers (new std::vector<int>);
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
        ransac.setDistanceThreshold (.075);
        ransac.computeModel();
        ransac.getInliers(*inliers);

        if (debug)
            std::cout<<"Laser callback cloud Begin1"<<std::endl;

        if (laser_cloud->size() >= MIN_LINE_SIZE){

            if (debug)
                std::cout<<"Laser callback cloud Begin2"<<std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

            if (debug)
                std::cout<<"Laser callback cloud Begin3"<<std::endl;
            pcl::copyPointCloud<pcl::PointXYZ>(*laser_cloud, *inliers, *final);

            if (debug)
                std::cout<<"Laser callback cloud Begin4"<<std::endl;
            if (final->points.size() >= MIN_LINE_SIZE)
                lineVector.push_back(final);
            std::cout<<"Cloud size before: "<<laser_cloud->size()<<std::endl;
            std::cout<<"Indice size: "<<inliers->size()<<std::endl;
            pcl::ExtractIndices<pcl::PointXYZ> eifilter (true);
            eifilter.setInputCloud(laser_cloud);
            eifilter.setNegative(true);
            eifilter.setIndices(inliers);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
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

    Line leftLine, rightLine;
    int leftMax = 0, rightMax = 0, leftMaxIndex = 0, rightMaxIndex = 0;
    for (int i = 0; i < lineVector.size(); i++){
        Line l = angle_lslf_cloud(lineVector.at(i));
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

    CloudView cv;
    cv.cloud = lineVector.at(leftMaxIndex);
    cv.flag = 4;
    cloudViewVector.push_back(cv);
    cv.cloud = lineVector.at(rightMaxIndex);
    cv.flag = 4;
    cloudViewVector.push_back(cv);

}

int main(int argc, char** argv){

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    std::string pcd_file = argv[filenames[0]];
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcd_file, *map_cloud);
    Laser_Callback_Cloud(map_cloud);
}
