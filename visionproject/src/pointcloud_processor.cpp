#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <image_transport/image_transport.h>
#include <pcl/visualization/cloud_viewer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>

class PointCloudProcessor {
public:
    PointCloudProcessor() {
        // ROS 뎁스 이미지와 카메라 정보 토픽 구독
        image_transport::ImageTransport it(nh);
        depth_sub = it.subscribeCamera("/camera/depth/image_raw", 1, &PointCloudProcessor::depthCallback, this);
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
        // 카메라 모델 설정
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(info_msg);

        // OpenCV로 뎁스 이미지 변환
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // 포인트 클라우드 생성
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        float fx = cam_model.fx(); // X축 초점 거리
        float fy = cam_model.fy(); // Y축 초점 거리
        float cx = cam_model.cx(); // X축 주점 좌표
        float cy = cam_model.cy(); // Y축 주점 좌표
        
        cout << "X축 초점 거리(fx): " << fx << endl;
        cout << "y축 초점 거리(fx): " << fy << endl;
        cout << "광학중심x축 거리: " << cx << endl;
        cout << "광학중심y축 거리: " << cy << endl;
        

        for (int i = 0; i < cv_ptr->image.rows; i++) {
            for (int j = 0; j < cv_ptr->image.cols; j++) {
                float z = cv_ptr->image.at<float>(i, j) / 1000.0f; // 뎁스 스케일을 미터 단위로 변환
                if (z > 0) {
                    pcl::PointXYZ point;
                    point.x = -(j - cx) * z / fx;
                    point.y = -(i - cy) * z / fy;
                    point.z = z;
                    cloud->points.push_back(point);
                }
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        // 포인트 클라우드 다운샘플링
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloud_filtered);

        // 포인트 클라우드 시각화
        viewer.showCloud(cloud_filtered);
    }

private:
    ros::NodeHandle nh;
    image_transport::CameraSubscriber depth_sub;
    pcl::visualization::CloudViewer viewer{"Point Cloud Viewer"};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_processor");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}

