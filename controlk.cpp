#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>

#include <iostream>
#include <algorithm>
#include <sstream>
#include<stdlib.h>
using namespace std;

const float PI = 3.14159265;
float rate = 100;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

class Rua {
public:
    int turtle_idx;
    ros::Subscriber sub;
    ros::Publisher pub;
    turtlesim::Pose current_pose;

    void callback(const turtlesim::Pose::ConstPtr& msg)
    {
        cout << "turtle " << turtle_idx+1 << " " << msg->x << " " << msg->y << endl;
        current_pose = *msg;
    }
};

double setDistance(turtlesim::Pose current_pose, double goal_x, double goal_y)
{
    double dis = sqrt(pow(goal_x - current_pose.x, 2) + pow(goal_y - current_pose.y, 2));
    if(dis < 0.01) dis = 0.0;
    return dis;
}

double setAngular(turtlesim::Pose current_pose, double goal_x, double goal_y)
{
    double ang;
    if(setDistance(current_pose, goal_x, goal_y) < 0.01) ang = 0.0;
    else
    {
        ang = asin((cos(current_pose.theta) * (goal_y - current_pose.y) - sin(current_pose.theta) * (goal_x - current_pose.x))
                    / setDistance(current_pose, goal_x, goal_y));
    }
    return ang;
}

double setGoc(turtlesim::Pose current_pose, double goal_x, double goal_y)
{
    double goc;
    goc = acos((cos(current_pose.theta)*(goal_x - current_pose.x)+sin(current_pose.theta)*(goal_y - current_pose.y))
                    / setDistance(current_pose, goal_x, goal_y));
    if(goc<PI / 2)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    

    int n_turtle = atoi(argv[1]);

    Rua rua[n_turtle];



    for(int i = 1; i < n_turtle; i++)
    {
        ros::service::waitForService("spawn");
        ros::ServiceClient spawner = h.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn turtle;
        turtle.request.x = rand() % 12;
        turtle.request.y = rand() % 12;
        turtle.request.theta = 0;
        spawner.call(turtle);
    }


    cout << "n_turtle = " << n_turtle << endl;
    for (int i = 0; i < n_turtle; i++) {
        stringstream s;
        s << "turtle" << i+1;
        string name = s.str();

        rua[i].turtle_idx = i;
        rua[i].sub = h.subscribe(name + "/pose", 100, &Rua::callback, &rua[i]);
        rua[i].pub = h.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 100);
        cout << "subcribe turtle " << i << " to " << name << "/pose" << endl;
    }
    ros::Rate loopRate(rate);

    double arr_goal[n_turtle][2];
    for(int idx = 0; idx < n_turtle; idx++)
    {
    	arr_goal[idx][0] = rua[idx].current_pose.x;
    	arr_goal[idx][1] = rua[idx].current_pose.y;
    }
    int id = 2;
    double tolerance=0.01;
    double a[n_turtle];

        while (ros::ok()) {
            for (int idx = 0; idx < n_turtle; idx++)
            {
                loopRate.sleep();
                ros::spinOnce();
                //cout << current_pose.x << " " << current_pose.y << " " << current_pose.theta << endl;

                if (setDistance(rua[idx].current_pose,arr_goal[idx][0],arr_goal[idx][1]) < tolerance) {
                    if(id < argc)
	                {
                        arr_goal[idx][0] = atof(argv[id++]);
                        arr_goal[idx][1] = atof(argv[id++]);
                        // cout << "turtle1: " << arr_goal[0][0] << " " << arr_goal[0][1] << endl;
                        a[idx]=setGoc(rua[idx].current_pose,arr_goal[idx][0],arr_goal[idx][1]);

                    }
                   
                }
                
               
                // a[idx]=setGoc(rua[idx].current_pose,arr_goal[idx][0],arr_goal[idx][1]);

                geometry_msgs::Twist msg = getMessage(
                    a[idx]*2.2*setDistance(rua[idx].current_pose,arr_goal[idx][0],arr_goal[idx][1]),
                    a[idx]*12.5*setAngular(rua[idx].current_pose,arr_goal[idx][0],arr_goal[idx][1])
                );

                rua[idx].pub.publish(msg);
            }
        }
    
    return 0;
}
