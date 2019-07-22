#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <chrono>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct PointLink{
    PointT point;
    PointLink* nextP = NULL;
    //PointT* preP;
};

struct GRID
{
    int traversability=0; /// 0:nothing 1:occupied
    int x, y;
    PointLink* pointHead = NULL;
    double height=0;
    double minHeight=100;
    /// histogram
};

class GridMap{
    float gridSize; ///unit in m
    GRID** map;
    float mapSize;
    int center;

public:
    void createMap(float grid_size, int map_size){
        gridSize = grid_size;
        mapSize = map_size;

        map = new GRID* [map_size];
        int i;
        //center = round(mapSize/2);

        for(i=0; i < map_size; i++)
        {
            map[i] = new GRID[map_size];
            /*for(int j=0; j<map_size; j++){
                GRID newGRID;
                map[i][j]=newGRID;
                map[i][j].traversability=0;
                map[i][j].y = i;
                map[i][j].x = j - center;
            }*/
        }
    }

    void labelTraversability(){

    }

    void computeHeightInGrid(int u, int v, double heightLimit){
        double total=0;
        double count=0;
        PointLink* curPL = map[v][u].pointHead;
        double minH=map[v][u].minHeight;
        while(curPL != NULL)
        {
            if(curPL->point.z >= minH+0.1 || curPL->point.z >=  heightLimit){
                total+=curPL->point.z;
                count+=1;
            }
            curPL=curPL->nextP;
        }
        if(count==0){
            map[v][u].height=-10;
        }
        else{
            map[v][u].height = total/count;
        }
    }

    double getColor(int u, int v){
        return map[v][u].height;
    }

    double getGridMinHeight(int u,int v){
        return map[v][u].minHeight;
    }
    void setGridMinHeight(int u,int v,double h){
        map[v][u].minHeight=h;
    }

    float getMapSize(){
        return mapSize;
    }

    float getGridSize(){
        return gridSize;
    }

    bool gridEmpty(int u, int v){
        return map[v][u].pointHead == NULL;
    }

    void insertPointToGrid(int u, int v, PointLink* PL){
        map[v][u].pointHead = PL;
    }

    PointLink* getPointInGrid(int u, int v){
        return map[v][u].pointHead;
    }
};

class VLP16
{
public:
    boost::shared_ptr<pcl::VLPGrabber> mVlpGrabber;

    boost::signals2::connection mConnection;

    pcl::PointCloud<PointT>::ConstPtr mCloud;

    boost::mutex mVLPMutex;

    void Start(string ipAddress, unsigned short port)
    {
        mVlpGrabber = boost::shared_ptr<pcl::VLPGrabber>(new pcl::VLPGrabber(boost::asio::ip::address::from_string(ipAddress), boost::lexical_cast<unsigned short>(port)));
        cout<<"0"<<endl;
        boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> cb = boost::bind(&VLP16::vlpCallback, this, _1);
        cout<<"1"<<endl;
        mConnection = mVlpGrabber->registerCallback(cb);
        cout<<"2"<<endl;
        mVlpGrabber->start();
        PointCloud::Ptr points=PointCloud::Ptr(new PointCloud());
        cout<<"3"<<endl;
    }
    void vlpCallback(const pcl::PointCloud<PointT>::ConstPtr& cloudPtr)
    {
        boost::mutex::scoped_lock lock(mVLPMutex);
        mCloud = cloudPtr;
    }
    void Close()
    {
        mVlpGrabber->stop();
        mConnection.disconnect();
    }
};


class ParameterReader
{
public:
    ParameterReader(string filename="../src/parameters.txt")
    {

        ifstream fin(filename.c_str());
        if(!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};


void toPCD(string infile);
std::queue<Eigen::Vector2i> LIDAR_Detect(int idx, ParameterReader pd);
