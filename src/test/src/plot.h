#ifndef PLOT_H
#define PLOT_H
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
using namespace std;

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

//Define custom
#define DEBUG_S1            0
#define DEBUG_S2            0
#define DEBUG_S3            0
#define DEBUG_S5            1

#define SENSOR_UPP          135.0        //unit: Degree
#define SENSOR_LOW          -135.0       //unit: Degree
#define SENSOR_SEGM         45.0         //unit: Degree for segmentation
#define SENSOR_RANGE        10.0         //unit: Degree same dircation
#define SENSOR_VIEW         0.75         //unit: m (for close distance[danger])

class XY
{
public:
    XY()
    {
        set(0,0);
    }

    XY(double x, double y)
    {
        set(x, y);
    }
    double x;
    double y;
    void set(double, double);
};

void XY::set(double ax, double ay)
{
    x=ax;
    y=ay;
}

class Plot
{
  public:
    Plot();
    Plot(double start_delay_time, double freq);
    ~Plot();
    Plot(std::string source_frame, std::string target_frame, double *out_x, double *out_y, double *out_z, double *out_yaw, bool *out_flag);
    //void init(double start_delay_time, double freq);
    void test_pub(void);
    void Polygon(int num, XY *xy, int ch_sel);
    void Polygon2(int num, XY *xy, double x0, double y0, int ch_sel);
    void Polygon_Line(int num, XY *xy, int ch_sel);

    double x;
  private:
    ros::NodeHandle nh_;
    ros::Publisher poly_pub1_;
    ros::Publisher poly_pub2_;
    ros::Publisher poly_pub3_;
    ros::Publisher poly_pub4_;
    ros::Publisher poly_pub5_;
    ros::Publisher poly_pub6_;
    ros::Publisher poly_pub7_;
    ros::Publisher poly_pub8_;
    ros::Publisher poly_pub9_;

    //std::string topic_name1;
    //std::string source_frame1;
};

Plot::Plot()
{
    poly_pub1_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub1", 50);
    poly_pub2_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub2", 50);
    poly_pub3_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub3", 50);
    poly_pub4_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub4", 50);
    poly_pub5_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub5", 50);
    poly_pub6_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub6", 50);
    poly_pub7_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub7", 50);
    poly_pub8_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub8", 50);
    poly_pub9_ = nh_.advertise<geometry_msgs::PolygonStamped>("/plot_pub9", 50);
}

Plot::~Plot()
{
}

void Plot::test_pub(void)
{
    geometry_msgs::Point32 poly;
    geometry_msgs::PolygonStamped pgon;

    pgon.header.frame_id="map";
    pgon.header.stamp=ros::Time::now();
    for(int i=0;i<10;i++)
    {
        poly.x=i;
        poly.y=i;
        pgon.polygon.points.push_back(poly);
    }
    poly_pub1_.publish(pgon);
}

void Plot::Polygon(int num, XY *xy, int ch_sel)
{
    geometry_msgs::Point32 poly;
    geometry_msgs::PolygonStamped pgon;

    pgon.header.frame_id="map";
    pgon.header.stamp=ros::Time::now();
    if (num>=1)
    {
        for(int i=0;i<num;i++)
        {
            poly.x=xy[i].x;
            poly.y=xy[i].y;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x-0.1;
            poly.y=xy[i].y+0.1;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x-0.1;
            poly.y=xy[i].y-0.1;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x+0.1;
            poly.y=xy[i].y-0.1;
            pgon.polygon.points.push_back(poly);
            poly.y=xy[i].x+0.1;
            poly.y=xy[i].y+0.1;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x;
            poly.y=xy[i].y;
            pgon.polygon.points.push_back(poly);
        }
    }
    else
    {
        //Clean Rviz Line
        poly.x=0;
        poly.y=0;
        pgon.polygon.points.push_back(poly);
    }

    if (ch_sel==0)
        poly_pub1_.publish(pgon);
    else if (ch_sel==1)
        poly_pub2_.publish(pgon);
    else if (ch_sel==2)
        poly_pub3_.publish(pgon);
    else if (ch_sel==3)
        poly_pub4_.publish(pgon);
    else if (ch_sel==4)
        poly_pub5_.publish(pgon);
    else if (ch_sel==5)
        poly_pub6_.publish(pgon);
    else if (ch_sel==6)
        poly_pub7_.publish(pgon);
    else if (ch_sel==7)
        poly_pub8_.publish(pgon);
    else if (ch_sel==8)
        poly_pub9_.publish(pgon);
}

void Plot::Polygon2(int num, XY *xy, double x0, double y0, int ch_sel)
{
    geometry_msgs::Point32 poly;
    geometry_msgs::PolygonStamped pgon;

    pgon.header.frame_id="map";
    pgon.header.stamp=ros::Time::now();
    if (num>=1)
    {
        for(int i=0;i<num;i++)
        {
            poly.x=xy[i].x;
            poly.y=xy[i].y;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x-0.1;
            poly.y=xy[i].y+0.1;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x-0.1;
            poly.y=xy[i].y-0.1;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x+0.1;
            poly.y=xy[i].y-0.1;
            pgon.polygon.points.push_back(poly);
            poly.y=xy[i].x+0.1;
            poly.y=xy[i].y+0.1;
            pgon.polygon.points.push_back(poly);
            poly.x=xy[i].x;
            poly.y=xy[i].y;
            pgon.polygon.points.push_back(poly);

            poly.x=x0;
            poly.y=y0;
            pgon.polygon.points.push_back(poly);
        }
    }
    else
    {
        //Clean Rviz Line
        poly.x=0;
        poly.y=0;
        pgon.polygon.points.push_back(poly);
    }

    if (ch_sel==0)
        poly_pub1_.publish(pgon);
    else if (ch_sel==1)
        poly_pub2_.publish(pgon);
    else if (ch_sel==2)
        poly_pub3_.publish(pgon);
    else if (ch_sel==3)
        poly_pub4_.publish(pgon);
    else if (ch_sel==4)
        poly_pub5_.publish(pgon);
    else if (ch_sel==5)
        poly_pub6_.publish(pgon);
    else if (ch_sel==6)
        poly_pub7_.publish(pgon);
    else if (ch_sel==7)
        poly_pub8_.publish(pgon);
    else if (ch_sel==8)
        poly_pub9_.publish(pgon);
}

void Plot::Polygon_Line(int num, XY *xy, int ch_sel)
{
    geometry_msgs::Point32 poly;
    geometry_msgs::PolygonStamped pgon;

    pgon.header.frame_id="map";
    pgon.header.stamp=ros::Time::now();
    if (num>=1)
    {
        for(int i=0;i<num;i++)
        {
            poly.x=xy[i].x;
            poly.y=xy[i].y;
            pgon.polygon.points.push_back(poly);
        }
    }
    else
    {
        //Clean Rviz Line
        poly.x=0;
        poly.y=0;
        pgon.polygon.points.push_back(poly);
    }

    if (ch_sel==0)
        poly_pub1_.publish(pgon);
    else if (ch_sel==1)
        poly_pub2_.publish(pgon);
    else if (ch_sel==2)
        poly_pub3_.publish(pgon);
    else if (ch_sel==3)
        poly_pub4_.publish(pgon);
    else if (ch_sel==4)
        poly_pub5_.publish(pgon);
    else if (ch_sel==5)
        poly_pub6_.publish(pgon);
    else if (ch_sel==6)
        poly_pub7_.publish(pgon);
    else if (ch_sel==7)
        poly_pub8_.publish(pgon);
    else if (ch_sel==8)
        poly_pub9_.publish(pgon);
}

//Dispaly Candidate Way-point Publish to Rviz
void Plot_Path_Rviz(int num, XY *xy, string path_str)
{
    int i;
    ros::NodeHandle nplot("~");
    ros::Publisher path_pub_;
    path_pub_ = nplot.advertise<nav_msgs::Path>(path_str, 50);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pp;

    path.header.frame_id="map";
    path.header.stamp=ros::Time::now();
    if (num>=1)
    {
        //origin point
        pp.pose.position.x=0;
        pp.pose.position.y=0;
        path.poses.push_back(pp);
        for(i=0;i<num;i++)
        {
            pp.pose.position.x=xy[i].x;
            pp.pose.position.y=xy[i].y;
            path.poses.push_back(pp);


            if (num==1)
            {
                pp.pose.position.x=xy[i].x+0.05;
                pp.pose.position.y=xy[i].y+0.05;
                path.poses.push_back(pp);
            }
        }
        path_pub_.publish(path);
    }
    else
    {
        //Clean Rviz Line
        pp.pose.position.x=0;
        pp.pose.position.y=0;
        path.poses.push_back(pp);
        path_pub_.publish(path);
    }
}

//Dispaly Candidate Way-point Publish to Rviz
void Plot_SPath_Rviz(int num, XY *xy, string path_str)
{
    int i;
    ros::NodeHandle nplot("~");
    ros::Publisher path_pub_;
    path_pub_ = nplot.advertise<nav_msgs::Path>(path_str, 50);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pp;

    path.header.frame_id="map";
    path.header.stamp=ros::Time::now();
    if (num>=1)
    {
        //origin point
        pp.pose.position.x=0;
        pp.pose.position.y=0;
        path.poses.push_back(pp);
        for(i=0;i<num;i++)
        {
            pp.pose.position.x=xy[i].x;
            pp.pose.position.y=xy[i].y;
            path.poses.push_back(pp);

            pp.pose.position.x=xy[i].x-0.1;
            pp.pose.position.y=xy[i].y+0.1;
            path.poses.push_back(pp);

            pp.pose.position.x=xy[i].x-0.1;
            pp.pose.position.y=xy[i].y-0.1;
            path.poses.push_back(pp);
            pp.pose.position.x=xy[i].x+0.1;
            pp.pose.position.y=xy[i].y-0.1;
            path.poses.push_back(pp);

            pp.pose.position.x=xy[i].x+0.1;
            pp.pose.position.y=xy[i].y+0.1;
            path.poses.push_back(pp);

            pp.pose.position.x=xy[i].x;
            pp.pose.position.y=xy[i].y;
            path.poses.push_back(pp);
        }
        path_pub_.publish(path);
    }
    else
    {
        //Clean Rviz Line
        pp.pose.position.x=0;
        pp.pose.position.y=0;
        path.poses.push_back(pp);
        path_pub_.publish(path);
    }
}

//Dispaly Footprint Polygon Publish to Rviz
void Plot_Polygon_Rviz(int num, XY *xy, string path_str)
{
    int i;
    ros::NodeHandle nplot("~");
    ros::Publisher poly_pub_;
    poly_pub_ = nplot.advertise<geometry_msgs::PolygonStamped>(path_str, 50);
    geometry_msgs::Point32 poly;
    geometry_msgs::PolygonStamped pgon;

    pgon.header.frame_id="map";
    pgon.header.stamp=ros::Time::now();
    if (num>=1)
    {
        for(i=0;i<num;i++)
        {
            poly.x=xy[i].x;
            poly.y=xy[i].y;
            pgon.polygon.points.push_back(poly);
        }
        poly_pub_.publish(pgon);
    }
    else
    {
        //Clean Rviz Line
        poly.x=0;
        poly.y=0;
        pgon.polygon.points.push_back(poly);
    }
    poly_pub_.publish(pgon);
}
#endif //PLOT_H
