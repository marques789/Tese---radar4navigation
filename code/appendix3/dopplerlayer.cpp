#include<my_layers/doppler_layer.h>
#include <pluginlib/class_list_macros.h>

//My Includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <my_layers/RadarPoint.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/message_filter.h>
#include <string>

#include <math.h>
#include <angles/angles.h>
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::DopplerLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

using my_layers::ObservationBuffer;
using my_layers::Observation;
using namespace tf;
using namespace std;

namespace simple_layer_namespace
{

DopplerLayer::DopplerLayer()
{}

void DopplerLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    default_value_ = FREE_SPACE;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<my_layers::DopplerPluginConfig>(nh);
    dynamic_reconfigure::Server<my_layers::DopplerPluginConfig>::CallbackType cb = boost::bind(
        &DopplerLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    //Parameters
    //Observation Buffer
    string topic="/xyzi_filt_out";
    double observation_keep_time=0.5;
    double expected_update_rate=10.0;
    double min_obstacle_height=-0.6;
    double max_obstacle_height=0.6;
    double obstacle_range=7.0;
    double raytrace_range=7.0;

    string global_frame_=layered_costmap_->getGlobalFrameID();
    string sensor_frame="";
    double transform_tolerance=0.5;
    observation_buffer_=
        boost::shared_ptr < ObservationBuffer
        > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                 max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                 sensor_frame, transform_tolerance));

    boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
        > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

    boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
        > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));

    filter->registerCallback(
        boost::bind(&DopplerLayer::pointCloud2Callback, this, _1, observation_buffer_));
    observation_subscriber_=sub;
    observation_notifier_=filter;

    //Cost function
    //amplitude=252.0;
    //cutoff=10.0;
    //covariance=0.03;
    //factor=20.0;
}


void DopplerLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
}


void DopplerLayer::reconfigureCB(my_layers::DopplerPluginConfig &config, uint32_t level)
{
      enabled_ = config.enabled;
      amplitude = config.amplitude;
      cutoff = config.cutoff;
      covariance = config.covariance;
      factor = config.factor;
}

void DopplerLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                                                        double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
        return;

    //Maybe change this?
    std::vector<Observation> observations;
    observation_buffer_->lock();
    observation_buffer_->getObservations(observations);
    observation_buffer_->unlock();
    bool update=true;
    for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
    {
        const Observation& obs = *it;
        const pcl::PointCloud<RadarPoint>& radar_cloud = *(obs.cloud_);
        for(unsigned int i=0; i < radar_cloud.points.size();++i){
            double doppler=radar_cloud.points[i].doppler;
            if(doppler<0.0){
                update=false;
                break;
            }
        }
    }
    //change to update the minimum area
    if(update){
        *min_x = -100.0;
        *min_y = -100.0;
        *max_x = 100.0;
        *max_y = 100.0;
    }
}

void DopplerLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                                                      int max_j)
{
    if (!enabled_)
        return;

    Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    string global_frame_=layered_costmap_->getGlobalFrameID();
    //string global_frame = costmap->getResolution();

    std::vector<Observation> observations;
    observation_buffer_->lock();
    observation_buffer_->getObservations(observations);
    observation_buffer_->unlock();
    for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
    {
        const Observation& obs = *it;
        const pcl::PointCloud<RadarPoint>& radar_cloud = *(obs.cloud_);
        for(unsigned int i=0; i < radar_cloud.points.size();++i){

            double wx = radar_cloud.points[i].x, wy = radar_cloud.points[i].y;
            //double wx = 1.0, wy = 1.0;
            double doppler=radar_cloud.points[i].doppler;
            //double doppler=0.24;

            if(doppler<0.0){

                //Convert map point to follow the costmap resolution
                unsigned int mx, my;
                if(costmap->worldToMap(wx, wy, mx, my)){
                }
                costmap->mapToWorld(mx,my,wx,wy);

                //Get sensor position  in the global frame
                geometry_msgs::PointStamped pt,opt;
                pt.header.frame_id="/base_radar_link";
                pt.point.x=0.0;
                pt.point.y=0.0;
                pt.point.z=0.0;

                tf_ -> transformPoint(global_frame_,pt,opt);
                double sensor_x,sensor_y;
                sensor_x=opt.point.x;
                sensor_y=opt.point.y;


                //Mark values around the point with radial velocity
                for (int j = -50; j < 50; j++)
                {
                    for (int i = -50; i < 50; i++)
                    {
                        //Get neighbour point global frame position
                        double x= wx+i*res, y=wy+j*res;

                        //Get position of the point relative to the sensor frame
                        double dx=wx-x,dy=wy-y;

                        //Angle of the neighbor point
                        double ma = atan2(dx,dy);
                        //Radial velocity angle converted to global frame
                        double radial_angle= atan2(wx-sensor_x,wy-sensor_y);
                        double diff = angles::shortest_angular_distance(radial_angle,ma);
                        if(fabs(diff)<M_PI/2 ){

                            //Distance from the evaluated point to the velocity point
                            double dist=sqrt(dx*dx+dy*dy);

                            double var_x=covariance*factor*fabs(doppler);
                            double var_y=covariance;
                            double x_t=cos(diff)*dist;
                            double y_t=sin(diff)*dist;
                            //because var_x> var_y then the values will be greater for
                            double f1=(pow(x_t,2.0))/(2.0*var_x);
                            double f2=(pow(y_t,2))/(2.0*var_y);

                            double factor=amplitude*exp(-(f1+f2));
                            unsigned char cost;
                            cost=factor;
                            if(cost<cutoff){
                                continue;
                            }
                            unsigned char cost_c= (unsigned char) cost;
                            unsigned int mx,my;
                            if(costmap->worldToMap(x,y,mx,my)){
                                unsigned char old_cost=costmap->getCost(mx,my);
                                costmap->setCost(mx,my,std::max(cost_c,old_cost));
                            }
                        }
                    }
                }
            }
        }
    }
 }
void DopplerLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                                                        const boost::shared_ptr<ObservationBuffer>& buffer)
{
    //buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(*message);
    buffer->unlock();
}
}
