/**
Copyright (C) 2016, by 
Feras Dayoub (feras.dayoub@gmail.com) 

This is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
 
This software package is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Leser General Public License.
If not, see <http://www.gnu.org/licenses/>.
**/

// whats this for??????????
#include "semantic_mapper/SemLabel.h"
#include <math.h>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
// cant find this header
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>

#include "message_filters/cache.h"
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>


#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap_types.h>
// why .h?????
#include "semantic_mapper/GetSemMap.h"



boost::mutex m_lock_;

typedef pcl::PCLPointCloud2 PclCloud;
typedef pcl::PCLPointCloud2::Ptr PclCloudPtr;
typedef sensor_msgs::PointCloud2 RosCloud;
typedef sensor_msgs::PointCloud2Ptr RosCloudPtr;


typedef octomap::ColorOcTreeNode SemCell;
typedef octomap::ColorOcTree SemMap;
typedef octomap::ColorOcTreeNode::Color CellColor;

#define Pr 0.299;
#define Pg 0.587;
#define Pb 0.114;

using namespace message_filters ;


class SemanticMapper{
    private:
        std::string inputLabelTopic_;
        std::string outputCloudTopic_;
        std::string laserTopic_;
        ros::NodeHandle nh_;
        ros::NodeHandle local_nh_;
        ros::Subscriber labelSub_;
        ros::Subscriber laserSub_;
        ros::Publisher cloudPub_;
        ros::Publisher oneLabel_pub_;
        sensor_msgs::Image image_;

        tf::TransformListener *tf_listener_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_cloud_ptr_;
        //bool send_semantic_cloud;
        laser_geometry::LaserProjection projector_;
        //int current_color[3];
        ros::Time label_time;

        message_filters::Cache<sensor_msgs::LaserScan> cache;

        SemMap *semMap;
        double cell_resolution_;
        double sem_ProbHit_;
        double sem_ProbMiss_;
        double prob_thres_;


    public:
        //constructor
        SemanticMapper(ros::NodeHandle nh,ros::NodeHandle local_nh):local_nh_(local_nh),nh_(nh)
        {
            tf_listener_ = new tf::TransformListener(local_nh_, ros::Duration(100));

            local_nh_.param<std::string>("label_topic",inputLabelTopic_,"/semantic_label");
            local_nh_.param<std::string>("output_cloud",outputCloudTopic_,"semantic_mapper/cloud");
            local_nh_.param<std::string>("laser_topic",laserTopic_,"laser");
            //local_nh_.param<std::string>("pc_topic",pcTopic_,"");
            local_nh_.param<double>("prob_thres",prob_thres_,0.5);

            labelSub_ = nh_.subscribe(inputLabelTopic_, 1,&SemanticMapper::label_CB,this);

            laserSub_ = nh_.subscribe(laserTopic_, 1,&SemanticMapper::Callback_laser,this);

            cloudPub_ = nh_.advertise<sensor_msgs::PointCloud2> (outputCloudTopic_,1);

            cache.setCacheSize(1000);
            cell_resolution_ = 0.1;
            semMap = new SemMap(cell_resolution_);
            semMap->setClampingThresMax(1.0);
            semMap->setOccupancyThres(0);

        }
        //constructor complete

         

        void label_CB(const semantic_mapper::SemLabel msg){
        // this const is not necessary
            boost::mutex::scoped_lock l(m_lock_);
            std::cout<< "messgage received" << std::endl;
            label_time = msg.header.stamp;
            sensor_msgs::LaserScan::ConstPtr laser_scan = cache.getElemBeforeTime(label_time);

            if ((laser_scan) && (msg.prob[msg.lvl] > prob_thres_)){
                std::cout << "prob" << msg.prob[msg.lvl] <<"\n";
                std::cout << "lvl" << msg.lvl <<"\n";
                pub_sem_cloud(laser_scan,msg.prob,msg.lvl,msg.r,msg.g,msg.b);
            }else{
                     std::cout << "no scan found" << "\n";
                }
            
            }


        void pub_sem_cloud(const sensor_msgs::LaserScan::ConstPtr& laser_scan,std::vector<float> prob,int lvl,std::vector<int> cr,std::vector<int> cg,std::vector<int> cb){


                std::cout << "scan received\n";
                
                sensor_msgs::LaserScan scan(*laser_scan);
                scan.header = laser_scan->header;

                for (int i=0; i < scan.ranges.size(); i++){
                    if ((scan.ranges[i] == 0)){
                          scan.ranges[i] = 3;
                    }

                }

                RosCloud input;
                projector_.projectLaser(scan, input);



                PclCloudPtr source(new PclCloud());
                pcl_conversions::toPCL(input,*source);
                pcl::PointCloud<pcl::PointXYZ>::Ptr raw_c (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr filled_c (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromPCLPointCloud2(*source, *raw_c);

                for( pcl::PointCloud<pcl::PointXYZ>::iterator iter = raw_c->begin(); iter != raw_c->end(); ++iter)
                {

                    float d = std::sqrt(iter->x*iter->x + iter->y*iter->y);
                    for (float i = 1/d; i < 1; i += 0.1)
                    {
                        float maxprob = prob[0]; 
                        float maxprobindex = 0;
                        for (int l=0;l < prob.size();l++)
                        {
                            if(maxprob < prob[l])
                            {
                                maxprob = prob[l]; 
                                maxprobindex = l;
                            }
                        }
                        pcl::PointXYZRGB p;
                        p.x = iter->x * i;
                        p.y = iter->y * i;
                        p.z = 0;  
                        uint8_t r = 0, g =0, b = 0;
                        r = cr[maxprobindex];
                        g = cg[maxprobindex];
                        b = cb[maxprobindex];
                        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                        p.rgb = *reinterpret_cast<float*>(&rgb);
                        filled_c->push_back(p);
                    }
                }


                tf::StampedTransform sensor_tf;
                tf_listener_->waitForTransform("map", "base_link",ros::Time::now(),ros::Duration(0.2));
                tf_listener_->lookupTransform("map", "base_link",laser_scan->header.stamp, sensor_tf);

                //sensor_tf.getRotation;
                filled_c->sensor_origin_[0] = sensor_tf.getOrigin().x();
                filled_c->sensor_origin_[1] = sensor_tf.getOrigin().y();
                filled_c->sensor_origin_[2] = sensor_tf.getOrigin().z();
                filled_c->sensor_orientation_.z() = sensor_tf.getRotation().z();
                filled_c->sensor_orientation_.w() = sensor_tf.getRotation().w();
                filled_c->sensor_orientation_.x() = sensor_tf.getRotation().x();
                filled_c->sensor_orientation_.y() = sensor_tf.getRotation().y();

                Eigen::Matrix3f rotMatrix = filled_c->sensor_orientation_.toRotationMatrix();
                Eigen::Matrix4f t;
                t(0, 0) = rotMatrix(0, 0);
                t(0, 1) = rotMatrix(0, 1);
                t(0, 2) = rotMatrix(0, 2);
                t(1, 0) = rotMatrix(1, 0);
                t(1, 1) = rotMatrix(1, 1);
                t(1, 2) = rotMatrix(1, 2);
                t(2, 0) = rotMatrix(2, 0);
                t(2, 1) = rotMatrix(2, 1);
                t(2, 2) = rotMatrix(2, 2);

                t(3, 0) = t(3, 1) = t(3, 2) = 0;
                t(3, 3) = 1;
                t(0, 3) = filled_c->sensor_origin_[0];
                t(1, 3) = filled_c->sensor_origin_[1];
                t(2, 3) = filled_c->sensor_origin_[2];
                pcl::transformPointCloud(*filled_c, *filled_c, t);
                
             
             for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = filled_c->begin(); it != filled_c->end(); it++){
                 octomap::point3d point(it->x, it->y, it->z);
                 semMap->updateNode(point,true);
                 SemCell* cell = semMap->search(point);
                 CellColor cS(it->r,it->g,it->b);
                 cell->setColor(cS);
             }


             pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOut(new pcl::PointCloud<pcl::PointXYZRGB>());
             for( SemMap::iterator itr = semMap->begin_leafs(), end = semMap->end_leafs(); itr != end; itr++){

                 SemCell *cell = semMap->search(itr.getCoordinate());
                 // for (int l=0;l < prob.size();l++){
                 //    SemCell *tcell = semMap->search(itr.getCoordinate().x(),itr.getCoordinate().y(),l);

                 //    if (tcell->getOccupancy() > cell->getOccupancy()){
                 //        cell = tcell;                        

                 //    }
                 // }
                 pcl::PointXYZRGB pcl_point;
                 pcl_point.x = itr.getCoordinate().x();pcl_point.y = itr.getCoordinate().y();pcl_point.z = 0;
                 CellColor c = cell->getColor();
                 pcl_point.r = c.r; pcl_point.g = c.g; pcl_point.b = c.b;
                 //this->changeSaturation(pcl_point.r,pcl_point.g,pcl_point.b, cell->getOccupancy());
                 pOut->push_back(pcl_point);
             }

             PclCloudPtr pclOut(new PclCloud());
             pcl::toPCLPointCloud2(*pOut,*pclOut);
             RosCloud outputCloud;
             pcl_conversions::fromPCL(*pclOut,outputCloud);
             outputCloud.header.frame_id = "map";
             outputCloud.header.stamp = laser_scan->header.stamp;
             cloudPub_.publish(outputCloud);

        }
        
        void Callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_scan){         
            cache.add(laser_scan);
        }


};
