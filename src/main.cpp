#include "lidar_cam.h"
#include "exlcm/publish_t.hpp"

class Handler
{
public:
    ~Handler() {}
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan)
    {
        int i;
        printf("Received message on channel \"%s\":\n", chan.c_str());
        cout<<rbuf<<endl;
    }
};



int main()
{
    lcm::LCM lcm;
    int frameNum = 1;
    string lcmNameHead = "ObstacleDetection LIDAR FRAME: ";
    ParameterReader pd;
    float gridSize=atof(pd.getData("gridSize").c_str());

    if(!lcm.good()){
        cout<<"lcm not good!"<<endl;
        return 1;
    }
    while(true){
        std::queue<Eigen::Vector2i> obstacleQ = LIDAR_Detect(frameNum, pd);
        if(obstacleQ.size() == 0)
        {
            cout<<"no obstacle detected!"<<endl;
            break;
            continue;
        }
        int queueSize = obstacleQ.size();
        cout<<"Num of Obstacles: "<<queueSize<<endl;
        while(!obstacleQ.empty())
        {
            exlcm::publish_t publishObstacleLocation;
            publishObstacleLocation.start = false;
            publishObstacleLocation.tail = false;
            if(queueSize == obstacleQ.size())
            {
                publishObstacleLocation.start = true;
            }
            else if(obstacleQ.size() == 1){
                publishObstacleLocation.tail = true;
            }
            Eigen::Vector2i obstacleLocation = obstacleQ.front();
            obstacleQ.pop();

            //publishObstacleLocation.timestamp = time(nullptr);
            publishObstacleLocation.name = lcmNameHead + to_string(frameNum);
            publishObstacleLocation.gridSize = gridSize;
            publishObstacleLocation.x = obstacleLocation(1);
            publishObstacleLocation.y = obstacleLocation(0);
            publishObstacleLocation.numObstacles = queueSize;

            lcm.publish("publishLIDAR", &publishObstacleLocation);
        }
        frameNum += 1;
    }
    /*VLP16 VLP;
    cout<<"good!"<<endl;
    VLP.Start("192.168.1.200", 2368);

    VLP.Close();*/

    return 0;
}
