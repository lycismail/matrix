#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "DataReceiver.h"
#include <time.h>
#include <vector>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <string>
#include <list>
#include <signal.h>
#include "protocol/meta.pb.h"
#include <condition_variable>
#include <memory>
#include <thread>
#include <tf/transform_listener.h>

#ifdef _WIN32
#include <windows.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <matrix_driver/HugoLineSegmentArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "YUVColorConvert.h"
using namespace std;
#define WORD short
#define MAX_PATH 256
typedef struct _SYSTEMTIME {
  WORD wYear;
  WORD wMonth;
  WORD wDayOfWeek;
  WORD wDay;
  WORD wHour;
  WORD wMinute;
  WORD wSecond;
  WORD wMilliseconds;
} SYSTEMTIME, *PSYSTEMTIME, *LPSYSTEMTIME;
#endif

timespec cond_wait_to_ = { 500, 1000 };

class AutoLock
{
public:
  AutoLock(pthread_mutex_t *mutex) {
    mutex_ = mutex;
    pthread_mutex_lock(mutex_);
  }
  ~AutoLock() {
    pthread_mutex_unlock(mutex_);
  }

private:
  pthread_mutex_t *mutex_;
};

const int imageWidth = 1280;
const int imageHeight = 720;

// thread for receive
static bool g_bStopReceive = false;
static bool g_bReceiveOver = false;
pthread_mutex_t g_lckReceiveOver;
pthread_cond_t g_condReceiveOver;

long long GetTimeStamp()
{
#ifdef _WIN32
  //LARGE_INTEGER freq, curr_time;
  //QueryPerformanceFrequency(&freq);
  //QueryPerformanceCounter(&curr_time);
  //return curr_time.QuadPart * 1000 / freq.QuadPart;
  {
    time_t tt = time(NULL);
    SYSTEMTIME systime;
    GetLocalTime(&systime);

    struct tm *tm_time;
    tm_time = localtime(&tt);

    return (long long)tt * 1000 + systime.wMilliseconds;
  }
#elif defined(__linux__) || defined(__ANDROID__) || defined(LINUX) || defined(ANDROID)
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  return ((long long)curr_time.tv_sec * 1000 + curr_time.tv_usec / 1000);

#elif defined(__MACH__)
#include <chrono>
  std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
  long long microsecs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  return microsecs;
#else
  REPORT_ERROR_POSITION;
#endif
}

SYSTEMTIME TimestampToSYSTEMTIME(long long timestamp) {

  int sec = timestamp / 1000;
  int msec = timestamp % 1000;

  time_t tt = sec;
  struct tm *tm_time;
  tm_time = localtime(&tt);

  SYSTEMTIME systime;
  systime.wYear = tm_time->tm_year + 1900;
  systime.wMonth = tm_time->tm_mon + 1;
  systime.wDay = tm_time->tm_mday;
  systime.wDayOfWeek = (tm_time->tm_wday + 7 - 1) % 7 + 1;
  systime.wHour = tm_time->tm_hour;
  systime.wMinute = tm_time->tm_min;
  systime.wSecond = tm_time->tm_sec;
  systime.wMilliseconds = msec;

  return systime;
}

#include <matrix_driver/Obstacle.h>
#include <matrix_driver/Obstacles.h>
#include <matrix_driver/Signal.h>
#include <matrix_driver/Signals.h>

#include <matrix_driver/ObstacleImgRect.h>
#include <matrix_driver/ObstacleImgRects.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <mutex>
#include <sensor_msgs/TimeReference.h>
#include <opencv_apps/RectArrayStamped.h>

ros::Time time_ref;
ros::Time time_matrix;
int seq_ref = 0;
ros::Time time_ros;
std::mutex time_mutex;
CommonProto::Point cvtImageToGround(const std::vector<float> &M_inv, const CommonProto::Point& pt) {
    CommonProto::Point g_pt;
    float z = M_inv[6] * pt.x() + M_inv[7] * pt.y() + M_inv[8];
    g_pt.set_x((M_inv[0] * pt.x() + M_inv[1] * pt.y() + M_inv[2]) / z);
    g_pt.set_y((M_inv[3] * pt.x() + M_inv[4] * pt.y() + M_inv[5]) / z);

    return g_pt;
}


// ros::Publisher pub_image;
// image_transport::Publisher pub_image;
image_transport::Publisher pub_mult1_image[8];

ros::Publisher pub_signal[8];
ros::Publisher pub_object[8];
// ros::Publisher pub_lane[8];
ros::Publisher lines_pub_custom[8];
//ros::Publisher bbox_pub[8];
ros::Publisher freespace_pub[8];
ros::Publisher freespace_marker_pub[8];
ros::Publisher object_marker_pub[8];
ros::Publisher roadline_marker_pub[8];

static YUVColorConvert *cvt = YUVColorConvert::GetInstance();

void ProcessMeta(FrameInfo *frame)
{
    ros::Time time;
    if (seq_ref == 0) {
        time = ros::Time::now();
    }
    else {
        time = time_ref + (ros::Time::now() - time_ros);
    }

    std::string coord("chassis_base");
    std::vector<float> minv_;

    std::cout<<"camera count"<<frame->meta.data().camera_matrix_size()<<std::endl;
    for(int k=0; k<frame->meta.data().camera_matrix_size();k++)
    {
        long int time_image = frame->meta.data().image(k).time_stamp();
//        int64_t time_image = frame_v1->image(k).time_stamp();
        double matrix_time_sec = time_image/1000.0;
        double ros_time_sec = ros::Time::now().toSec();
        double used_time_sec;

        if(fabs(ros_time_sec - matrix_time_sec) < 1.0){
          used_time_sec = matrix_time_sec;
        } else{
          used_time_sec = ros_time_sec;
          cout << "\033[31mTime sync with matrix is failed, ros time is used:  "
               << setprecision(15)<< "matrix itme: " << matrix_time_sec
               << ", ros time:" << setprecision(15) << ros_time_sec << "\033[0m" << endl;

        }
        time_matrix = ros::Time::Time(used_time_sec);
        minv_.clear();
        for (int i = 0; i < frame->meta.data().camera_matrix(k).mat_img2vcsgnd_size(); ++i)
        {
            minv_.push_back(frame->meta.data().camera_matrix(k).mat_img2vcsgnd(i));
        }
        matrix_driver::Obstacles msg_objs;
        matrix_driver::Signals msg_sigs;

        msg_objs.obs.resize(frame->meta.data().structure_perception().obstacles(k).obstacle_size());


        //msg_objs.header.seq = ;
        msg_objs.header.stamp = time;
        msg_objs.header.frame_id = coord;

        msg_sigs.header.stamp = time;
        msg_sigs.header.frame_id = coord;


        std::cout<< frame->meta.data().frame_id() << ": pub objects " \
                 << frame->meta.data().structure_perception().obstacles(k).obstacle_size() << std::endl;

        matrix_driver::ObstacleImgRects msg_bbox;

        msg_bbox.header.stamp = time;
        msg_bbox.header.frame_id = "matrix"+std::to_string(frame->meta.data().structure_perception().obstacles(k).cam_id());
        msg_bbox.obs.resize(frame->meta.data().structure_perception().obstacles(k).obstacle_size());
        int lightcount=0;
        visualization_msgs::MarkerArray object_array;
        for (int i = 0; i < frame->meta.data().structure_perception().obstacles(k).obstacle_size(); ++i)
        {
            std::cout<<"obs count"<<frame->meta.data().structure_perception().obstacles(k).obstacle_size()<<std::endl;
            matrix_driver::Obstacle &msg_obj = msg_objs.obs[i];
            msg_obj.header.frame_id = coord;
            msg_obj.header.stamp = time;
            msg_obj.Classification = frame->meta.data().structure_perception().obstacles(k).obstacle(i).type();

            msg_obj.ObsId = frame->meta.data().structure_perception().obstacles(k).obstacle(i).id();
            msg_obj.Life = frame->meta.data().structure_perception().obstacles(k).obstacle(i).life_time();
            cout<<"msg_obj.Classification : "<<msg_obj.Classification<<endl;
            if(4==msg_obj.Classification)
            {
                matrix_driver::Signal msg_sig;
                msg_sig.id=to_string(msg_obj.ObsId);
                if(frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_type_size()>0){
                    int type_size=frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_type_size();
                    for(int j=0;j<type_size;j++){
                        if(frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_type(j)==0){
                            msg_sig.type=frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_name(j);
                        }
                        else if(frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_type(j)==1){
                            std::string obs_property=frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_name(j);
                            if("green"==string(obs_property)){
                                msg_sig.color=2;
                            }
                            else if("yellow"==string(obs_property)){
                                msg_sig.color=1;
                            }
                            else if("red"==string(obs_property)){
                                msg_sig.color=0;
                            }
                            else{
                                msg_sig.color=0xff;
                            }
                        }
                    }
                }
                else{
                    if(frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_name_size()>0){
                        std::string obs_property=frame->meta.data().structure_perception().obstacles(k).obstacle(i).property_name(0);

                        if("green"==string(obs_property)){
                            msg_sig.color=2;
                        }
                        else if("yellow"==string(obs_property)){
                            msg_sig.color=1;
                        }
                        else if("red"==string(obs_property)){
                            msg_sig.color=0;
                        }
                        else{
                            msg_sig.color=0xff;
                        }
                    }

                }

                msg_sig.image_left = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().left();
                msg_sig.image_top = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().top();
                msg_sig.image_right = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().right();
                msg_sig.image_bottom = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().bottom();
//                int right_bottom_x = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().right();
//                int right_bottom_y = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().bottom();

                msg_sig.Width = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().width();
                msg_sig.Height = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().height();
                msg_sig.WorldPosition.x=frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().position().x();
                msg_sig.WorldPosition.y=frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().position().y();
                msg_sig.WorldPosition.z=frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().position().z();

                msg_sigs.signals.push_back(msg_sig);
                lightcount++;
            }
             CommonProto::Point left, right;
             left.set_x(frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().left());
             left.set_y(frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().top());
             right.set_x(frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().right());
             right.set_y(frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().bottom());
             CommonProto::Point gleft, gright;
             gleft = cvtImageToGround(minv_, left);
             gright = cvtImageToGround(minv_, right);
             msg_obj.Width = gleft.y() - gright.y();
             switch(msg_obj.Classification)
             {
                 case 2:
                         msg_obj.Length=0.5;
                         break;
                 default:
                         msg_obj.Length=5;
                         break;
             }

//            msg_obj.Length = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().length();
//            msg_obj.Width = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().width();
            msg_obj.Height = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().height();
            msg_obj.image_left = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().left();
            msg_obj.image_top = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().top();
            msg_obj.image_right=frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().right();
            msg_obj.image_bottom=frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().bottom();

            msg_obj.ObsTheta = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().yaw();



            msg_obj.ObsPosition.x = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().position().x();
            msg_obj.ObsPosition.y = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().position().y();
            msg_obj.ObsPosition.z = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().position().z();
            msg_obj.Velocity.x = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().vel().vx();
            msg_obj.Velocity.y = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().vel().vy();
            msg_obj.Velocity.z = frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().vel().vz();

            visualization_msgs::Marker object_cube;
            object_cube.header.frame_id = coord;
            object_cube.header.stamp = time;
            object_cube.id = i;
            object_cube.action = visualization_msgs::Marker::ADD;
            object_cube.type = visualization_msgs::Marker::CUBE;
            object_cube.lifetime = ros::Duration(0.1) ;

            object_cube.pose.position.x = msg_obj.ObsPosition.x;
            object_cube.pose.position.y = msg_obj.ObsPosition.y;
            object_cube.pose.position.z = msg_obj.ObsPosition.z;
            tf::Quaternion q;
            q.setRPY(0, 0, frame->meta.data().structure_perception().obstacles(k).obstacle(i).world_info().yaw());
            object_cube.pose.orientation.x = q[0];
            object_cube.pose.orientation.y = q[1];
            object_cube.pose.orientation.z = q[2];
            object_cube.pose.orientation.w = q[3];
            object_cube.scale.x = msg_obj.Length;
            object_cube.scale.y = msg_obj.Width;
            object_cube.scale.z = 0.1;
            if( msg_obj.Classification == 1)
            {
                object_cube.color.a = 1.0; // Don't forget to set the alpha!
                object_cube.color.r = 0.0;
                object_cube.color.g = 1.0;
                object_cube.color.b = 0.0;
            }
            else if( msg_obj.Classification == 2)
            {
                object_cube.color.a = 1.0; // Don't forget to set the alpha!
                object_cube.color.r = 1.0;
                object_cube.color.g = 0.0;
                object_cube.color.b = 0.0;
            }
            else
            {
                object_cube.color.a = 1.0; // Don't forget to set the alpha!
                object_cube.color.r = 0.0;
                object_cube.color.g = 0.0;
                object_cube.color.b = 1.0;
            }
            object_array.markers.push_back(object_cube);




//            matrix_driver::ObstacleImgRect &bb = msg_bbox.obs[i];
//            bb.x = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().left();
//            bb.y = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().top();
//            bb.width = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().right() - \
//                       frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().left();
//            bb.height = frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().bottom() - \
//                        frame->meta.data().structure_perception().obstacles(k).obstacle(i).img_info().rect().top();
//            bb.Classification=msg_obj.Classification;
        }


        if(frame->meta.data().structure_perception().obstacles(k).has_cam_id())
        {
            if(lightcount>0)
            {
                pub_signal[frame->meta.data().structure_perception().obstacles(k).cam_id()].publish(msg_sigs);
            }
            pub_object[frame->meta.data().structure_perception().obstacles(k).cam_id()].publish( msg_objs );
//            bbox_pub[frame->meta.data().structure_perception().obstacles(k).cam_id()].publish( msg_bbox );
            std::cout<<"obs count"<<frame->meta.data().structure_perception().obstacles(k).obstacle_size()<<std::endl;
            std::cout << "object count" << object_array.markers.size();
            object_marker_pub[frame->meta.data().structure_perception().obstacles(k).cam_id()].publish( object_array );
        }
        else
        {
            std::cout<<"please confirm cam_id value"<<std::endl;
        }

        if( frame->meta.data().structure_perception().lines_size() > 0){
            if( frame->meta.data().structure_perception().lines(k).lines_size() > 0 )
            {
                int all_n = 0;
                for (int i = 0; i < frame->meta.data().structure_perception().lines(k).lines_size(); ++i) {
                    if (frame->meta.data().structure_perception().lines(k).lines(i).end_points_size() != 2 || \
                        frame->meta.data().structure_perception().lines(k).lines(i).coeffs_size() != 4) {
                        continue;
                    }

                    all_n += fabs(frame->meta.data().structure_perception().lines(k).lines(i).end_points(0).y() -\
                               frame->meta.data().structure_perception().lines(k).lines(i).end_points(1).y()) + 1;
                }
                std::cout<< frame->meta.data().frame_id() << ": pub lane " << all_n << std::endl;

                visualization_msgs::Marker line_list;
                line_list.header.frame_id = coord;
                line_list.header.stamp = time;
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.pose.orientation.w = 1.0;
                line_list.id = k;
                //use only the x component of scale, for the line width
                line_list.scale.x = 0.1;
                line_list.color.r = 1.0;
                line_list.color.b = 1.0;
                line_list.color.a = 1.0;


                matrix_driver::HugoLineSegmentArray lines;
                lines.header.frame_id = coord;
                lines.header.stamp = time;
                lines.arraylength = frame->meta.data().structure_perception().lines(k).lines_size();
                lines.HugoLineSegmentArray.resize(lines.arraylength);
                lines.coeffs_num = 4;

                int n = 0;
                for (int i = 0; i < frame->meta.data().structure_perception().lines(k).lines_size(); ++i)
                {
                    if (frame->meta.data().structure_perception().lines(k).lines(i).end_points_size() != 2 || \
                        frame->meta.data().structure_perception().lines(k).lines(i).coeffs_size() != 4)
                    {
                        continue;
                    }

                    float xs = frame->meta.data().structure_perception().lines(k).lines(i).end_points(1).x();
                    float xe = frame->meta.data().structure_perception().lines(k).lines(i).end_points(0).x();
                    float a = frame->meta.data().structure_perception().lines(k).lines(i).coeffs(0);
                    float b = frame->meta.data().structure_perception().lines(k).lines(i).coeffs(1);
                    float c = frame->meta.data().structure_perception().lines(k).lines(i).coeffs(2);
                    float d = frame->meta.data().structure_perception().lines(k).lines(i).coeffs(3);
                    // float ys = xs;
                    // float ye = xe;

                    lines.HugoLineSegmentArray[i].line_coeffs.resize(lines.coeffs_num);
                    lines.HugoLineSegmentArray[i].line_coeffs[0] = a;
                    lines.HugoLineSegmentArray[i].line_coeffs[1] = b;
                    lines.HugoLineSegmentArray[i].line_coeffs[2] = c;
                    lines.HugoLineSegmentArray[i].line_coeffs[3] = d;

                    lines.HugoLineSegmentArray[i].start_point.x = xs;
                    lines.HugoLineSegmentArray[i].start_point.y = a + b * xs + c * xs * xs + d * xs * xs * xs;
                    lines.HugoLineSegmentArray[i].start_point.z = 0.0;

                    lines.HugoLineSegmentArray[i].end_point.x = xe;
                    lines.HugoLineSegmentArray[i].end_point.y = a + b * xe + c * xe * xe + d * xe * xe * xe;
                    lines.HugoLineSegmentArray[i].end_point.z = 0.0;

                    lines.HugoLineSegmentArray[i].line_type = frame->meta.data().structure_perception().lines(k).lines(i).type();
                    lines.HugoLineSegmentArray[i].life_time = frame->meta.data().structure_perception().lines(k).lines(i).life_time();
                    lines.HugoLineSegmentArray[i].dist_to_front_wheel = frame->meta.data().structure_perception().lines(k).lines(i).dist_to_front_wheel();
                    

                    geometry_msgs::Point p;
                    p.x = lines.HugoLineSegmentArray[i].start_point.x;
                    p.y = lines.HugoLineSegmentArray[i].start_point.y;
                    p.z = lines.HugoLineSegmentArray[i].start_point.z;
                    line_list.points.push_back(p);
                    geometry_msgs::Point q;
                    q.x = lines.HugoLineSegmentArray[i].end_point.x;
                    q.y = lines.HugoLineSegmentArray[i].end_point.y;
                    q.z = lines.HugoLineSegmentArray[i].end_point.z;
                    line_list.points.push_back(q);

                }

                if(frame->meta.data().structure_perception().obstacles(k).has_cam_id())
                {
                    // pub_lane[frame->meta.data().structure_perception().obstacles(k).cam_id()].publish(msg_lane);
                    lines_pub_custom[frame->meta.data().structure_perception().obstacles(k).cam_id()].publish(lines);
                    roadline_marker_pub[frame->meta.data().structure_perception().obstacles(k).cam_id()].publish(line_list);
                }
                else
                {
                    std::cout<<"please confirm cam_id value"<<std::endl;
                }
            }
        }

        if( frame->meta.data().structure_perception().scan_pts_size() >0)
        {
            int pts_number = 0;
            pts_number = frame->meta.data().structure_perception().scan_pts(k).pts_vcs_size();
            std::cout << frame->meta.data().frame_id() << ": pub pointsss" << pts_number << std::endl;

            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = coord;
            line_strip.header.stamp = time;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = k;
            //use only the x component of scale, for the line width
            line_strip.scale.x = 0.1;
            line_strip.color.r = 1.0;
            line_strip.color.g = 1.0;
            line_strip.color.a = 1.0;

            sensor_msgs::PointCloud2 msg_frsp;
            msg_frsp.header.frame_id = coord;
            msg_frsp.header.stamp = time;
            msg_frsp.height = 1;
            msg_frsp.width = pts_number;
            msg_frsp.point_step = 20;
            msg_frsp.row_step = pts_number * msg_frsp.point_step;
            msg_frsp.is_dense = false;
            msg_frsp.is_bigendian = false;
            msg_frsp.data.clear();
            msg_frsp.fields.resize( 5 );

            msg_frsp.fields[0].name = "x";
            msg_frsp.fields[0].offset = 0;
            msg_frsp.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            msg_frsp.fields[0].count = 1;

            msg_frsp.fields[1].name = "y";
            msg_frsp.fields[1].offset = 4;
            msg_frsp.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            msg_frsp.fields[1].count = 1;

            msg_frsp.fields[2].name = "z";
            msg_frsp.fields[2].offset = 8;
            msg_frsp.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            msg_frsp.fields[2].count = 1;

            msg_frsp.fields[3].name = "rgb";
            msg_frsp.fields[3].offset = 12;
            msg_frsp.fields[3].datatype = sensor_msgs::PointField::FLOAT32 ;
            msg_frsp.fields[3].count = 1;

            msg_frsp.fields[4].name = "type";
            msg_frsp.fields[4].offset = 16;
            msg_frsp.fields[4].datatype = sensor_msgs::PointField::INT32;
            msg_frsp.fields[4].count = 1;

            float *p = NULL;
            char *q = (char*)p;
            for(int i = 0; i < pts_number; ++i)
            {
                float  point_x = float(frame->meta.data().structure_perception().scan_pts(k).pts_vcs(i).x());
                p = &point_x;
                q = (char*)p;
                msg_frsp.data.push_back(*q);
                msg_frsp.data.push_back(*(q+1));
                msg_frsp.data.push_back(*(q+2));
                msg_frsp.data.push_back(*(q+3));

                float  point_y = float(frame->meta.data().structure_perception().scan_pts(k).pts_vcs(i).y());
                p = &point_y;
                q = (char*)p;
                msg_frsp.data.push_back(*q);
                msg_frsp.data.push_back(*(q+1));
                msg_frsp.data.push_back(*(q+2));
                msg_frsp.data.push_back(*(q+3));

                float  point_z = float(frame->meta.data().structure_perception().scan_pts(k).pts_vcs(i).z());
                p = &point_z;
                q = (char*)p;
                msg_frsp.data.push_back(*q);
                msg_frsp.data.push_back(*(q+1));
                msg_frsp.data.push_back(*(q+2));
                msg_frsp.data.push_back(*(q+3));

                int point_type = frame->meta.data().structure_perception().scan_pts(k).property(i);

                int r=1,g=1,b=1,rgb=0;
                switch (point_type)
                {
                  case 4://
                    //cout << "enter color 4" << endl;
                    r=255;
                    g=0;
                    b=0;
                    break;
                  case 1://
                    //cout << "enter color 1" << endl;
                    r=0;
                    g=255;
                    b=0;
                    break;
                  case 255://
                    //cout << "enter color 255" << endl;
                    r=0;
                    g=0;
                    b=255;
                    break;
                  default:
                    //cout << "enter color default" << endl;
                    r=255;
                    g=255;
                    b=0;
                    break;
                }
                rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
                int *p_rgb = NULL;
                char *q_rgb = NULL;
                p_rgb = &rgb;
                q_rgb = (char*)p_rgb;

                msg_frsp.data.push_back(*q_rgb);
                msg_frsp.data.push_back(*(q_rgb+1));
                msg_frsp.data.push_back(*(q_rgb+2));
                msg_frsp.data.push_back(*(q_rgb+3));

                int *p_type = NULL;
                char *q_type = NULL;
                p_type = &point_type;
                q_type = (char*)p_type;

                msg_frsp.data.push_back(*q_type);
                msg_frsp.data.push_back(*(q_type+1));
                msg_frsp.data.push_back(*(q_type+2));
                msg_frsp.data.push_back(*(q_type+3));

                geometry_msgs::Point point;
                point.x = point_x;
                point.y = point_y;
                point.z = point_z;
                line_strip.points.push_back(point);

            }

            if(frame->meta.data().structure_perception().scan_pts(k).has_cam_id())
            {
                // std::cout << frame->meta.data().structure_perception().scan_pts(k).cam_id() << std::endl;
                freespace_pub[frame->meta.data().structure_perception().scan_pts(k).cam_id()].publish(msg_frsp);
                freespace_marker_pub[frame->meta.data().structure_perception().scan_pts(k).cam_id()].publish(line_strip);
            }
        }
    }
}

void ProcessFrame(FrameInfo *frame) {
    ros::Time time;
//    if (seq_ref == 0) {
//        time = ros::Time::now();
//    }
//    else {
//        time = time_ref + (ros::Time::now() - time_ros);
//    }

    sensor_msgs::ImagePtr msg;
    int img_size = 0;

    const char *g_image_format[] = {
      "Gray",
      "YV12",
      "JPEG",     // jpeg compressed
      "PNG",      // png compressed
      "CR12",     // reserved
      "BAD",      // reserved
      "NV12",
      "NV21",
      "Timeout"   // timeout image
    };

    static int subsample_rate[] = {
        1, 1, 2, 4, 1
    };

    for (int i = 0; i < frame->img_data.size(); i++){
        long int time_image_frame = frame->meta.data().image(i).time_stamp();
        double matrix_time_sec_frame = time_image_frame/1000.0;
        double ros_time_sec_frame = ros::Time::now().toSec();
        double used_time_sec_frame;

        if(fabs(ros_time_sec_frame - matrix_time_sec_frame) < 1.0){
          used_time_sec_frame = matrix_time_sec_frame;
        } else{
          used_time_sec_frame = ros_time_sec_frame;
          cout << "\033[31mTime sync with matrix is failed, ros time is used in frame~~~~:  "
               << setprecision(15)<< "matrix itme: " << matrix_time_sec_frame
               << ", ros time:" << setprecision(15) << ros_time_sec_frame << "\033[0m" << endl;

        }
        time = ros::Time::Time(used_time_sec_frame);
        const CommonProto::Image &img_info = frame->meta.data().image(i);
        uint32_t width = img_info.width();
        uint32_t height = img_info.height();
        int sub_sample = img_info.send_mode();
        int scale = subsample_rate[sub_sample];
        int color_mode = img_info.format();    //for matrix, NV12 and jpeg only

        std::cout << "Image " << i << ": ("
          << width << ", "
          << height << ", "
          << scale << ", "
          << g_image_format[color_mode] << ")" << std::endl;

        cv::Mat image;
        if (color_mode == CommonProto::ImageFormat::TIMEOUT)
        {
          // this is a fake image when camera interrupt timeout happened
        }
        else if (color_mode == CommonProto::ImageFormat::JPEG)
        {
          // sample to get jpeg image form raw data
            img_size = width * height * 3 / 2;
            cv::Mat im_stream(1, frame->img_data[i].size(), CV_8UC1,
                                frame->img_data[i].data());
            image = cv::imdecode(im_stream, cv::IMREAD_COLOR);
        }
        else
        {
            uint8_t *img_data = frame->img_data[i].data();
            uint8_t *img_y = img_data;
            uint8_t *img_uv = img_data + width * height;  // uvuvuv pattern
            img_size = (width * height * 3) >> 1;
            // sample for a gray image
            cv::Mat mat_gray(height, width, CV_8UC1, img_data, width);

            // sample for resize back to origin image size
            cv::Mat mat_resize(height * scale, width * scale, mat_gray.type());
            cv::resize(mat_gray, mat_resize, mat_resize.size());
            image = mat_resize;
        }

        std_msgs::Header header=std_msgs::Header();
        header.stamp=time;
        header.frame_id="image"+to_string(i);
        msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        pub_mult1_image[i].publish(msg);
    }

    ProcessMeta(frame);
}

void * ThreadRecieve(void *ip_param) {
    std::string server = (char *)ip_param;

    std::cout<<"IP:PORT: " << server << std::endl;
    DataReceiver mq_sub;
    std::string end_point;
    if (server.substr(0, 3) != "tcp") {
        end_point = "tcp://";
        end_point += server;
    }
    else {
        end_point = server;
    }
    mq_sub.init(end_point.c_str());

    unsigned int mq_count = 0;

    float avg_fps = 0;

    bool bRecvFail = false;
    int nRecvFailCount = 0;
    while (!g_bStopReceive)
    {
        std::cout<< "Runniing...." << std::endl;
        FrameInfo *frame = new FrameInfo;
//        memset(frame, 0, sizeof(FrameInfo));

        int tic = GetTimeStamp();
        if (!mq_sub.RecvFrame(frame))
        {
          {
            bRecvFail = true;
            nRecvFailCount++;

            if (nRecvFailCount > 20) {
              mq_sub.reconnect();
              nRecvFailCount = 0;
            }
          }
          continue;
        }

    // TODO: deal with meta info
        ProcessFrame(frame);


        int toc = GetTimeStamp();

        bRecvFail = false;
        nRecvFailCount = 0;
        mq_count++;

        int time = toc - tic;
        tic = toc;
        if (time > 0) {
          avg_fps = avg_fps * 0.9 + 1000.0f / time * 0.1;
        }

        if (mq_count % 20 == 0) {
          printf("current: %.3fFPS, avg: %.3f\n",
                  1000.0f / time, avg_fps);
        }
        delete frame;
    } // end of while

    g_bReceiveOver = true;
    pthread_cond_broadcast(&g_condReceiveOver);

    return 0;
}

void timeCallback(const sensor_msgs::TimeReferenceConstPtr& msg) {
    time_mutex.lock();
    time_ref = msg->time_ref;
    seq_ref = msg->header.seq;
    time_ros = ros::Time::now();
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(9)
            << "GPS_Timestamp: " << time_ref.toSec();
    time_mutex.unlock();
}

void mysig()
{
    sleep(1);
    exit(0);
}
int main(int argc, char **argv){
    char *server = "192.168.0.12:5560";
    ros::init(argc, argv, "matrix_driver", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    std::string hugo_ip;
    if (n.getParam("/matrix/ip", hugo_ip)) server = hugo_ip.c_str();
    std::cout << "seted server: " << server << std::endl;

    signal(SIGINT,mysig);
    image_transport::ImageTransport it(n);
    ros::Subscriber time_sub = n.subscribe("/sensor/gnss/time_reference", 1, &(timeCallback));
    // pub_image = it.advertise("/sensor/matrix/image", 1);

    pub_mult1_image[0] = it.advertise("/sensor/matrix/image1", 1);
    pub_mult1_image[1] = it.advertise("/sensor/matrix/image2", 1);
    pub_mult1_image[2] = it.advertise("/sensor/matrix/image3", 1);
    pub_mult1_image[3] = it.advertise("/sensor/matrix/image4", 1);

    pub_signal[0]=n.advertise<matrix_driver::Signals>("/sensor/matrix/signals1",1);
    pub_object[0] = n.advertise<matrix_driver::Obstacles>("/sensor/matrix/objects1", 1);
    // pub_lane[0]= n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/lane1", 1);
    lines_pub_custom[0]= n.advertise<matrix_driver::HugoLineSegmentArray>("/sensor/matrix/lines1", 1);
//    bbox_pub[0] = n.advertise<matrix_driver::ObstacleImgRects>("/sensor/matrix/bbox1", 1);
    freespace_pub[0] = n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/freespace1", 1);
    freespace_marker_pub[0] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_freespace1", 1);
    //freespace_point_marker_pub[0] = n.advertise<visualization_msgs::Marker>("sensor/matrix/Marker_freespace_point1", 1);
    object_marker_pub[0] = n.advertise<visualization_msgs::MarkerArray>("/sensor/matrix/Marker_object1", 1);
    roadline_marker_pub[0] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_roadline1", 1);

    pub_signal[1]=n.advertise<matrix_driver::Signals>("/sensor/matrix/signals2",1);
    pub_object[1]= n.advertise<matrix_driver::Obstacles>("/sensor/matrix/objects2", 1);
    // pub_lane [1]= n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/lane2", 1);
    lines_pub_custom [1] = n.advertise<matrix_driver::HugoLineSegmentArray>("/sensor/matrix/lines2", 1);
//    bbox_pub[1] = n.advertise<matrix_driver::ObstacleImgRects>("/sensor/matrix/bbox2", 1);
    freespace_pub[1] = n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/freespace2", 1);
    freespace_marker_pub[1] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_freespace2", 1);
    //freespace_point_marker_pub[1] = n.advertise<visualization_msgs::Marker>("sensor/matrix/Marker_freespace_point2", 1);
    object_marker_pub[1] = n.advertise<visualization_msgs::MarkerArray>("/sensor/matrix/Marker_object2", 1);
    roadline_marker_pub[1] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_roadline2", 1);

    pub_signal[2]=n.advertise<matrix_driver::Signals>("/sensor/matrix/signals3",1);
    pub_object[2]= n.advertise<matrix_driver::Obstacles>("/sensor/matrix/objects3", 1);
    // pub_lane[2]= n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/lane3", 1);
    lines_pub_custom[2]= n.advertise<matrix_driver::HugoLineSegmentArray>("/sensor/matrix/lines3", 1);
//    bbox_pub[2]= n.advertise<matrix_driver::ObstacleImgRects>("/sensor/matrix/bbox3", 1);
    freespace_pub[2] = n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/freespace3", 1);
    freespace_marker_pub[2] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_freespace3", 1);
    //freespace_point_marker_pub[2] = n.advertise<visualization_msgs::Marker>("sensor/matrix/Marker_freespace_point3", 1);
    object_marker_pub[2] = n.advertise<visualization_msgs::MarkerArray>("/sensor/matrix/Marker_object3", 1);
    roadline_marker_pub[2] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_roadline3", 1);

    pub_signal[3]=n.advertise<matrix_driver::Signals>("/sensor/matrix/signals4",1);
    pub_object[3]= n.advertise<matrix_driver::Obstacles>("/sensor/matrix/objects4", 1);
    // pub_lane[3]= n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/lane4", 1);
    lines_pub_custom[3]= n.advertise<matrix_driver::HugoLineSegmentArray>("/sensor/matrix/lines4", 1);
//    bbox_pub[3]= n.advertise<matrix_driver::ObstacleImgRects>("/sensor/matrix/bbox4", 1);
    freespace_pub[3] = n.advertise<sensor_msgs::PointCloud2>("/sensor/matrix/freespace4", 1);
    freespace_marker_pub[3] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_freespace4", 1);
    //freespace_point_marker_pub[3] = n.advertise<visualization_msgs::Marker>("sensor/matrix/Marker_freespace_point4", 1);
    object_marker_pub[3] = n.advertise<visualization_msgs::MarkerArray>("/sensor/matrix/Marker_object4", 1);
    roadline_marker_pub[3] = n.advertise<visualization_msgs::Marker>("/sensor/matrix/Marker_roadline4", 1);

    ros::Rate rate(10.0);
    // init mutex and conditional variables
    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);

    pthread_condattr_t condattr;
    pthread_condattr_init(&condattr);

    pthread_mutex_init(&g_lckReceiveOver, &mutexattr);
    pthread_cond_init(&g_condReceiveOver, &condattr);

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);

    pthread_t th;
    g_bStopReceive = false;
    g_bReceiveOver = false;
   pthread_create(&th, &thread_attr, ThreadRecieve, server);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
    g_bStopReceive = true;

    {
    AutoLock lck(&g_lckReceiveOver);
    if (!g_bReceiveOver) {
      pthread_cond_wait(&g_condReceiveOver, &g_lckReceiveOver);
    }
    }

    google::protobuf::ShutdownProtobufLibrary();

    pthread_attr_destroy(&thread_attr);
    pthread_mutexattr_destroy(&mutexattr);
    pthread_condattr_destroy(&condattr);

    return 0;
}

