#include "lidar_cam.h"

void toPCD(string infile)
{
    int i,index;
    string outfile, fname,filename;

    for(index=0; index<157; index++)
    {
        outfile="pcd/"+to_string(index)+".pcd";
        // load point cloud
        ifstream input;
        if(index<10){
            fname="000000000"+to_string(index);
        }
        else if(index<100){
            fname="00000000"+to_string(index);
        }
        else{
            fname="0000000"+to_string(index);
        }
        filename=infile+fname+".txt";
        input.open(infile+fname+".txt");
        char num[sizeof(double)];

        PointCloud::Ptr points=PointCloud::Ptr(new PointCloud());
        for (i=0; input.is_open() && !input.eof(); i++) {
            PointT point;

            input>>num;
            point.x=atof(num);
            input>>num;
            point.y=atof(num);
            input>>num;
            point.z=atof(num);

            input>>num;

            if(point.x>=0){
                point.r=255;
                points->push_back(point);
            }
        }
        input.close();
        cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile << endl;
        pcl::io::savePCDFile(outfile,*points);
    }
    cout<<"Done converting!"<<endl;
}


void LIDAR_Detect()
{
    ParameterReader pd;
    string file_path = pd.getData("PCD_Path").c_str();
    float gridSize=atof(pd.getData("gridSize").c_str());
    int mapSize=atoi(pd.getData("mapSize").c_str());
    int limit=atoi(pd.getData("LIDAR_rangeLimit").c_str()); /// unit m
    float LIDAR_height=atof(pd.getData("LIDAR_Height").c_str());
    float PassAbility=atof(pd.getData("PassAbility").c_str());
    float vehicleHeight = 1;

    GridMap map;
    map.createMap(gridSize, mapSize);
    int center = round(mapSize/2);

    ///load pointcloud
    PointCloud::Ptr cloud=PointCloud::Ptr(new PointCloud());
    if(pcl::io::loadPCDFile<PointT>(file_path,*cloud)==-1)
    {
        cout<<"Error reading file "<<file_path<<endl;
        return;
    }

    int u, v;

    //map to points to gridMap
    PointCloud::Ptr newPoints=PointCloud::Ptr(new PointCloud());
    for(int i=0; i<cloud->size(); i++){
        PointT point = cloud->points[i];
        if(point.x>=limit || point.y >= round(limit/2) || point.y <= -round(limit/2))
            continue;
        u = floor(point.y/gridSize);
        v = floor(point.x/gridSize);

        PointLink* PL = new PointLink;
        PL->point = point;
        u=u+center;

        if(map.gridEmpty(u, v)){
            map.insertPointToGrid(u, v, PL);
        }
        else{
            PL->nextP = map.getPointInGrid(u, v);
            map.insertPointToGrid(u, v, PL);
            if(point.z < map.getGridMinHeight(u,v)){
                map.setGridMinHeight(u,v,point.z);
            }
        }
    }

    //compute height histogram for each grid
    for(int i=0;i<mapSize;i++){
        for(int j=-center; j<center;j++){

            int c=j+center;
            int r=i;
            map.computeHeightInGrid(c,r,(-LIDAR_height+PassAbility));
            PointLink* PL=map.getPointInGrid(c,r);
            double height=(map.getColor(c,r));
            while(PL!=NULL){
                PointT p=PL->point;
                if(height>= (-LIDAR_height+PassAbility)){
                    p.r=255;
                    p.g=0;
                    p.b=0;
                }
                else{
                    p.g=255;
                    p.r=0;
                    p.b=0;
                }
                newPoints->push_back(p);
                PL=PL->nextP;
            }
        }
    }
    cout << "new points, writing to "<< endl;
    pcl::io::savePCDFile("outfile.pcd",*newPoints);
}





















