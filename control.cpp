#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include <algorithm>
using namespace std;

ros::Publisher pub;
const float PI = 3.14159265;
float rate = 100;
turtlesim::Pose current_pose;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    ros::Time start=ros::Time::now();
    pub = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub =
        h.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);

   

    for(int i=1; i<argc; i+=2)
    {
        double x0=atof(argv[i]);
        double y0=atof(argv[i+1]);
        const double tolerance = 0.1;

        while (ros::ok()) {
            loopRate.sleep();
            ros::spinOnce();
            cout << current_pose.x << " " << current_pose.y << " " << current_pose.theta << endl;

            
            double distance = sqrt( pow(x0-current_pose.x, 2) + pow(y0-current_pose.y, 2) );
            if (distance < tolerance) { 
                pub.publish(getMessage(0,0));
                break;
            }
            // double alpha = atan2( y0-current_pose.y, x0-current_pose.x ),a_z;
            
            double dx = x0 - current_pose.x, dy = y0 - current_pose.y, theta = current_pose.theta;
           
            double dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);
            double goc=acos((cos(theta)*dx+sin(theta)*dy) / distance);
            
            if((goc<PI/2) && (goc>-PI/2))
            {
                geometry_msgs::Twist msg = getMessage(
                    7*distance,
                    14*dalpha
            );

                pub.publish(msg);
            }
            else
            {
                geometry_msgs::Twist msg = getMessage(
                    -7*distance,
                    -14*dalpha
            );

            pub.publish(msg);
            }
        }
    }
    ros::Time finish = ros::Time::now();
    cout<<"total time:" << (finish-start).toSec();
    return 0;
}

