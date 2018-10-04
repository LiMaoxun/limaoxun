#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "plot.h"//show data on Rviz
#include "comm.h"


//ROS Parameter
#define MAIN_FREQ          100.0      //Controller main frequency
#define TF_REV_DELAY_T     3.0        //TF of MAP_ODOM frame receive start delay units * ts
#define LIDAR_POSE_X       0.275      //Coodinate of lidar to base_link

double main_freq_=MAIN_FREQ ;
double tf_rev_delay_t_=TF_REV_DELAY_T;
double lidar_pose_x_=LIDAR_POSE_X;

int num_new=0;//number of points with range less than 5m
float scan_range[3000];//Lidar: reduce the number of scan points (delete the points with the range over 5m)
float scan_angle[3000];
int start[1081];//Lidar scan: first point index of each group
int end[1081];//Lidar scan: final point index of each group
float scan_x[3000];//Lidar frame: PolToRec x
float scan_y[3000];//Lidar frame: PolToRec y
double scan_pose_x[3000];//Base_link frame x
double scan_pose_y[3000];//Base_link frame y
int num_endpoint=0;//number of endpoints (start and end)
double scan_pose_startpoint_x[300];
double scan_pose_startpoint_y[300];
double scan_pose_endpoint_x[300];
double scan_pose_endpoint_y[300];
double dis_cal[1081];//distance between two points from two adjacent group
double dis_min[300];//shortest distance between two adjacent group
double dis_min_min[300];
int min_point_idx[1081];//k
int min_point_idx_next[1081];
int min_group_idx[300];//j
int min_group_idx_next[300];
int check_min_point_idx[300];
int check_min_point_idx_next[300];
int num_corri_point=0;//number of pair of possible narrow entry

//Plot ploygan
XY scan_points[3000];
XY scan_startpoints[300];
XY scan_endpoints[300];
XY scan_corri_startpoints[100];
XY scan_corri_endpoints[100];

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int num=0;
    num=((msg->angle_max-msg->angle_min)/msg->angle_increment)+1;//number of scan cloud points
    //ROS_INFO("I heard: [%d]", num);
    //ROS_INFO("I heard: [%f]", msg->ranges[0]);

    int j=0;
    for(int i=0;i<num;i++)
    {
        if(msg->ranges[i]<5.0)
        {
            scan_range[j]=msg->ranges[i];//r
            scan_angle[j]=msg->angle_min+i*(msg->angle_increment);//theta=-135degree+increment
            //PolToRec
            scan_x[j]=scan_range[j]*cos(scan_angle[j]);//x
            scan_y[j]=scan_range[j]*sin(scan_angle[j]);//y
            scan_pose_x[j]=scan_x[j]+lidar_pose_x_;//Base_link frame x
            scan_pose_y[j]=scan_y[j];//Base_link frame y
            j++;
        }
    }
    num_new=j;

    //seperate groups for narrow corridor detection; Here we record the index of start point and end point of each group
    j=0;
    start[j]=0;
    for(int i=0;i<num_new-1;i++)
    {
        //if(scan_range[i+1]-scan_range[i]>0.65)
        if(DIS_XY(scan_pose_x[i],scan_pose_y[i],scan_pose_x[i+1],scan_pose_y[i+1])>0.65)//the width of the vehicle is 0.65
           {
            end[j]=i;
            start[j+1]=i+1;
            j++;
           }
    }
    end[j]=num_new-1;
    num_endpoint=j+1;//number of start point or end point

    //record the coordinates of the start&end points
    for(int i=0;i<num_endpoint;i++)
    {
        scan_pose_startpoint_x[i]=scan_pose_x[(unsigned int) start[i]];
        scan_pose_startpoint_y[i]=scan_pose_y[(unsigned int) start[i]];
        scan_pose_endpoint_x[i]=scan_pose_x[(unsigned int) end[i]];
        scan_pose_endpoint_y[i]=scan_pose_y[(unsigned int) end[i]];
    }

    //calculate the shortest distance between two adjacent groups
    int i=0;
    int k=0;

    for(i=0;i<num_endpoint-1;i++)//from 1st group to last group
    {
        int gp1_num=end[i]-start[i]+1;
        j=0;
        while(j<gp1_num)//previous group
        {
            int gp2_num=end[i+1]-start[i+1]+1;
            k=0;
            while(k<gp2_num)//next group
            {
               //double x1,y1,x2,y2;
/*
               dis_cal[k]=DIS_XY(scan_pose_x[start[i]+j],scan_pose_y[start[i]+j],scan_pose_x[start[i+1]+k],scan_pose_y[start[i+1]+k]);
               if(k=0)
                   dis_min[j]=dis_cal[k];
               if(k>0)
               {
                   if(dis_cal[k]>dis_cal[k-3])
                   {
                       dis_min[j]=dis_cal[k-3];
                       min_point_idx[j]=j+1;
                       min_point_idx_next[j]=k-3+1;
                   }
                   else
                   {
                       dis_min[j]=dis_cal[k];
                       min_point_idx[j]=j+1;
                       min_point_idx_next[j]=k+1;
                   }
               }
               k=k+3;

            }
            if(j=0)
                dis_min_min[i]=dis_min[j];
            if(j>0)
            {
                if(dis_min[j]>dis_min[j-3])
                {
                    dis_min_min[i]=dis_min[j-3];
                    min_group_idx[i]=min_point_idx[j-3];
                    min_group_idx_next[i]=min_point_idx_next[j-3];
                }
                else
                {
                    dis_min_min[i]=dis_min[j];
                    min_group_idx[i]=min_point_idx[j];
                    min_group_idx_next[i]=min_point_idx_next[j];
                }
            }
            j=j+3;
            */
        }

    }

    j=0;
    k=0;
    if(i=num_endpoint-1)//between 1st group and last group
    {
        while(j<end[i]-start[i]+1)//previous group
        {
            while(k<end[0]-start[0]+1)//next group
            {
               dis_cal[k]=DIS_XY(scan_pose_x[start[i]+j],scan_pose_y[start[i]+j],scan_pose_x[start[0]+k],scan_pose_y[start[0]+k]);
               if(k=0)
                   dis_min[j]=dis_cal[k];
               if(k>0)
               {
                   if(dis_cal[k]>dis_cal[k-3])
                   {
                       dis_min[j]=dis_cal[k-3];
                       min_point_idx[j]=j+1;
                       min_point_idx_next[j]=k-3+1;
                   }
                   else
                   {
                       dis_min[j]=dis_cal[k];
                       min_point_idx[j]=j+1;
                       min_point_idx_next[j]=k+1;
                   }
               }
               k=k+3;

            }
            if(j=0)
                dis_min_min[i]=dis_min[j];
            if(j>0)
            {
                if(dis_min[j]>dis_min[j-3])
                {
                    dis_min_min[i]=dis_min[j-3];
                    min_group_idx[i]=min_point_idx[j-3];
                    min_group_idx_next[i]=min_point_idx_next[j-3];
                }
                else
                {
                    dis_min_min[i]=dis_min[j];
                    min_group_idx[i]=min_point_idx[j];
                    min_group_idx_next[i]=min_point_idx_next[j];
                }
            }
            j=j+3;
        }
    }
    int ii=0;
    for(i=0;i<num_endpoint;i++)
    {
        if((dis_min_min[i]<4*0.65) && (dis_min_min[i]>1.2*0.65))//the width of the vehicle is 0.65
        {
            check_min_point_idx[ii]=min_point_idx[i];
            check_min_point_idx_next[ii]=min_point_idx_next[i];
            ii++;
        }

    }
    num_corri_point=ii+1;//number of pair of possible narrow entry
    */
}


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "scan_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/base_scan", 10, scanCallback);

    //==================================================
    //load ROS Parameter
    n.getParam("/main_freq", main_freq_) ;
    n.getParam("/tf_rev_delay_t", tf_rev_delay_t_);
    n.getParam("/lidar_pose_x", lidar_pose_x_);

    ros::Rate loop_rate(main_freq_);      //100HZ

    //Get Pose from TF listener
    GetTFPose tf1(tf_rev_delay_t_, main_freq_); //Receive tf (SLAM or Localization)  /map  <---/base_link data
    //tf::TransformBroadcaster broadcaster;

    Plot pt_group_point;

    printf("~~!!start_scan_node\n");

     while(ros::ok())
    {
        //v main loop v ==============================================================
        for(int i=0;i<num_new;i++)//XY
            scan_points[i].set(scan_pose_x[i],scan_pose_y[i]);

        for(int i=0;i<num_endpoint;i++)//XY
        {
            scan_startpoints[i].set(scan_pose_startpoint_x[i],scan_pose_startpoint_y[i]);
            scan_endpoints[i].set(scan_pose_endpoint_x[i],scan_pose_endpoint_y[i]);
        }

        for(int i=0;i<num_corri_point;i++)//XY
        {
            scan_corri_startpoints[i].set(scan_pose_x[check_min_point_idx[i]],scan_pose_y[check_min_point_idx[i]]);
            scan_corri_endpoints[i].set(scan_pose_x[check_min_point_idx_next[i]],scan_pose_y[check_min_point_idx_next[i]]);
        }

        if(num_new>1)//show the following points on Rviz
        {
            pt_group_point.Polygon(num_new, &scan_points[0], 6);//ch_sel=6 means to use  topic /plot_pub7; give first point, then the function will calculate the following points
            pt_group_point.Polygon(num_endpoint, &scan_startpoints[0], 7);//topic /plot_pub8
            pt_group_point.Polygon(num_endpoint, &scan_endpoints[0], 8);//topic /plot_pub9
            //pt_group_point.Polygon(num_corri_point, &scan_corri_startpoints[0], 7);
            //pt_group_point.Polygon(num_corri_point, &scan_corri_endpoints[0], 8);

        }

        printf("~~!!num_new=%d\n", num_new);
        //^ main loop ^================================================================
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
