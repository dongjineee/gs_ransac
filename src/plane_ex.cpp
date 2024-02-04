#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include <random>
#include <boost/filesystem.hpp>

#include <math.h>
namespace fs = boost::filesystem;
class PointCloudConverter {
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub_1;
  ros::Publisher pub_2;
  ros::Publisher pub_3;
  ros::Publisher pub_4;
  ros::Publisher pub_5;
  ros::Publisher pub_6;
  std::vector<std::string> pcd_file_list_;
  std::string frame_id;

// "frame_id" 파라미터가 설정되어 있지 않으면, 기본값으로 "test"를 사용
  double start_time, end_time, time,sum;

  #define ground 0 
  #define y_wall 1
  #define x_wall 2
  #define not_plane 33

  int cnt = 0;


public:
  PointCloudConverter(){
    // Subscriber와 Publisher 설정
    pub_ = nh_.advertise<sensor_msgs::PointCloud>("/converted_pointcloud_topic", 1);
    sub_ = nh_.subscribe("/velodyne_points", 1, &PointCloudConverter::pointCloud2Callback, this);

    pub_1 = nh_.advertise<sensor_msgs::PointCloud2>("/test", 1);
    pub_2 = nh_.advertise<sensor_msgs::PointCloud2>("/test_1", 1);
    pub_3 = nh_.advertise<sensor_msgs::PointCloud2>("/test_2", 1);
    pub_4 = nh_.advertise<sensor_msgs::PointCloud2>("/test_3", 1);
    pub_5 = nh_.advertise<sensor_msgs::PointCloud2>("/test_4", 1);
    pub_6 = nh_.advertise<sensor_msgs::PointCloud2>("/test_5", 1);

    nh_.param<std::string>("frame_id", frame_id, "ouster_lidar");

    process_pcd_files("/home/dongjin/lidar_pr_data_set_1/pcd_file");  // PCD 파일 처리 메서드 호출

  }

    int extractNumber(const std::string& filename) {
        std::string number;
        for (auto c : filename) {
            if (isdigit(c)) number += c;
        }
        return number.empty() ? 0 : std::stoi(number);
    }
    void process_pcd_files(const std::string& directory_path) {
        fs::path p(directory_path);
        std::vector<fs::path> pcd_files;

        // 디렉토리 내의 모든 파일 순회하며 PCD 파일 경로 저장
        for (fs::directory_iterator itr(p); itr != fs::directory_iterator(); ++itr) {
            if (fs::is_regular_file(itr->path()) && itr->path().extension() == ".pcd") {
                pcd_files.push_back(itr->path());
            }
        }

        // 파일 이름에 포함된 숫자를 기준으로 오름차순 정렬
        std::sort(pcd_files.begin(), pcd_files.end(), [&](const fs::path& a, const fs::path& b) {
            return extractNumber(a.filename().string()) < extractNumber(b.filename().string());
        });

        // 정렬된 파일 순서대로 처리
        for (auto& file_path : pcd_files) {
            process_single_pcd_file(file_path.string());
            ros::Duration(1.0).sleep();
        }
    }


    void process_single_pcd_file(const std::string& pcd_file) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *loaded_cloud) == -1) {
            PCL_ERROR("Couldn't read file %s\n", pcd_file.c_str());
            return;  // 현재 파일만 처리하고 종료
        }
        
        // 로드된 클라우드에 대한 필요한 처리 함수 호출
         ransac(*loaded_cloud); // 이 부분은 ransac 함수의 구현에 따라 달라집니다.
    }

  // void process_pcd_files() {
  //   // PCD 파일 리스트를 반복하면서 처리
  //   for (const std::string& pcd_file : pcd_file_list_) {
  //     process_single_pcd_file(pcd_file);
  //   }
  // }

  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
    // PointCloud2를 pcl::PointCloud로 변환
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*input_cloud, pcl_cloud);
    
    ransac(pcl_cloud);
  }

  void ransac(const pcl::PointCloud<pcl::PointXYZ>& input_cloud)
  {
    // 필터링
    start_time = ros::Time::now().toSec();

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud ;
    filtered_cloud = filter_cloud(input_cloud, 0.15, Eigen::Vector4f(-20, -6, -2, 1), Eigen::Vector4f(30, 7, 5, 1));

    sensor_msgs::PointCloud2 test_cloud;  
    test_cloud.header.frame_id = frame_id;
    test_cloud.header.stamp = ros::Time::now();

    pcl::toROSMsg(filtered_cloud, test_cloud);
    pub_1.publish(test_cloud);

    std::pair<std::vector<uint8_t>, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> segmentResult;
    segmentResult = seg_plane<pcl::PointXYZ>(filtered_cloud.makeShared(), 250, 0.05, 5);

    end_time = ros::Time::now().toSec();
    time = end_time-start_time;
    sum += time;

    //ROS_INFO("%f",time);
    start_time = ros::Time::now().toSec();
    // Convert the first PointCloud to ROS message and publish
    sensor_msgs::PointCloud2 test_1_cloud;
 
    pcl::toROSMsg(*(segmentResult.second[0]), test_1_cloud);
    test_1_cloud.header.frame_id = frame_id;
    test_1_cloud.header.stamp = ros::Time::now();

    pub_2.publish(test_1_cloud);

    sensor_msgs::PointCloud2 test_2_cloud;
 
    pcl::toROSMsg(*(segmentResult.second[1]), test_2_cloud);
    test_2_cloud.header.frame_id = frame_id;
    test_2_cloud.header.stamp = ros::Time::now();

    pub_3.publish(test_2_cloud);

    sensor_msgs::PointCloud2 test_3_cloud;
 
    pcl::toROSMsg(*(segmentResult.second[2]), test_3_cloud);
    test_3_cloud.header.frame_id = frame_id;
    test_3_cloud.header.stamp = ros::Time::now();

    pub_4.publish(test_3_cloud);

    sensor_msgs::PointCloud2 test_4_cloud;
 
    pcl::toROSMsg(*(segmentResult.second[3]), test_4_cloud);
    test_4_cloud.header.frame_id = frame_id;
    test_4_cloud.header.stamp = ros::Time::now();

    pub_5.publish(test_4_cloud);

    sensor_msgs::PointCloud2 test_5_cloud;
 
    pcl::toROSMsg(*(segmentResult.second[4]), test_5_cloud);
    test_5_cloud.header.frame_id = frame_id;
    test_5_cloud.header.stamp = ros::Time::now();

    pub_6.publish(test_5_cloud);

/*
    sensor_msgs::PointCloud output_cloud;
    output_cloud.header.frame_id = "test";
    output_cloud.header.stamp = ros::Time::now();
    output_cloud.points.resize(filtered_cloud.size());

    for (size_t i = 0; i < filtered_cloud.size(); ++i) {
      output_cloud.points[i].x = filtered_cloud.points[i].x;
      output_cloud.points[i].y = filtered_cloud.points[i].y;
      output_cloud.points[i].z = filtered_cloud.points[i].z;
    }

    // 변환된 PointCloud를 발행
    pub_.publish(output_cloud);
*/ 
//pcl2sen_msgs

    //ROS_INFO("t1: %d, t2: %d, t3: %d, t4: %d, t5; %d, cnt : %d", segmentResult.first[0],segmentResult.first[1],segmentResult.first[2],segmentResult.first[3],segmentResult.first[4],cnt);

  }

  pcl::PointCloud<pcl::PointXYZ> filter_cloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
  {
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud.makeShared());
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(filtered_cloud);
    //std::cerr << "Voxeled " << filtered_cloud.points.size () << std::endl;

    pcl::CropBox<pcl::PointXYZ> roi;
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(filtered_cloud.makeShared()); 
    roi.filter(filtered_cloud);
    //std::cerr << "ROI " << filtered_cloud.points.size () << std::endl;
    
    //Remove all the points from roof
    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZ> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud(filtered_cloud.makeShared());
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point:indices)
    	inliers->indices.push_back(point);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(filtered_cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(filtered_cloud); //천장에 대한 point 제거
    //std::cerr << "extract " << filtered_cloud.points.size () << std::endl;
    return filtered_cloud;
  }

uint8_t plane_jurdgment( Eigen::Vector3f normal_vector)
{
    normal_vector.normalize(); // 법선 벡터 정규화

    Eigen::Vector3f z_axis_pos(0, 0, 1);
    Eigen::Vector3f y_axis_pos(0, 1, 0);
    Eigen::Vector3f x_axis_pos(1, 0, 0);
    Eigen::Vector3f z_axis_nag(0, 0, -1);
    Eigen::Vector3f y_axis_nag(0, -1, 0);
    Eigen::Vector3f x_axis_nag(-1, 0, 0);

    std::vector<Eigen::Vector3f> axis = {z_axis_pos, y_axis_pos, x_axis_pos, z_axis_nag ,y_axis_nag,x_axis_nag};

    const float angleThreshold = 5.0 * M_PI / 180.0; // 5도를 라디안으로 변환

    for (int i = 0; i < 6; i++) {

        float angle = acos(normal_vector.dot(axis[i])); // 두 벡터 간의 각도 계산, 내적 계산

        if (abs(angle) <= angleThreshold) {
            return i; // 조건을 만족하는 축에 해당하는 포지션 반환
            
        }

    }

    
    return 33; // 어떤 축과도 매치되지 않는 경우
}

// 2개 변수 return, vector<pcl:XYZ> -> plane에 관한 PCD, vector<int> -> plane에 관한 id 0: ground, 1 : y-wall, 2 : x-wall
template<typename PointXYZ>
std::pair<std::vector<uint8_t>, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>  seg_plane(
    typename pcl::PointCloud<PointXYZ>::Ptr input_cloud, 
    int maxIterations, 
    float distanceThreshold, 
    uint8_t planes_num)
{

    std::vector<typename pcl::PointCloud<PointXYZ>::Ptr> planes;
    std::vector<uint8_t> plane_types;

    typename pcl::PointCloud<PointXYZ>::Ptr remainingCloud = input_cloud; //초기 input cloud

    uint8_t plane_types_data;

    std::vector<Eigen::Vector3f> end_normal_vector;
    uint8_t plane_num = 0;

    for(int k = 0; k < planes_num; k++) {

        std::unordered_set<int> inliersResult;
        int largestPlaneSize = 0;
        Eigen::Vector3f bestNormal(0, 0, 0);

        uint8_t type;
        std::vector<uint8_t> cost_seg{0,0,0}; 


        for(int it = 0; it < maxIterations; it++) {
            std::unordered_set<int> tempIndices;
            while(tempIndices.size() < 3) {
                tempIndices.insert(rand() % remainingCloud->points.size());
            }

            auto iter = tempIndices.begin();
            PointXYZ point1 = remainingCloud->points[*iter]; ++iter;
            PointXYZ point2 = remainingCloud->points[*iter]; ++iter;
            PointXYZ point3 = remainingCloud->points[*iter];

            float a = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
            float b = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
            float c = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
            float d = -(a * point1.x + b * point1.y + c * point1.z);

            for(int index = 0; index < remainingCloud->points.size(); index++) {
                if(tempIndices.find(index) == tempIndices.end()) {

                    PointXYZ point = remainingCloud->points[index];

                    float distance = fabs(a * point.x + b * point.y + c * point.z + d) / sqrt(a * a + b * b + c * c);

                    if(distance <= distanceThreshold) {
                        tempIndices.insert(index); // 평면에 속하는 pc의 index를 집어 넣음
                    }

                }
            }

            if(tempIndices.size() > largestPlaneSize) {
                largestPlaneSize = tempIndices.size();
                inliersResult = tempIndices;
                bestNormal = Eigen::Vector3f(a, b, c);  // 법선 벡터 업데이트
            } //평면이 계속 커지는 느낌

        } 
      
        
        if(largestPlaneSize == 0) {
            break; // 더 이상 평면을 찾을 수 없음
        }
        
        typename pcl::PointCloud<PointXYZ>::Ptr planeCloud(new pcl::PointCloud<PointXYZ>());
        typename pcl::PointCloud<PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<PointXYZ>());

        for(int index = 0; index < remainingCloud->points.size(); index++) {
            PointXYZ point = remainingCloud->points[index];
            if(inliersResult.find(index) != inliersResult.end())
                planeCloud->points.push_back(point);
            else
                cloudOutliers->points.push_back(point); // input_data update ( outliners )
        }

        Eigen::Vector3f bestNormal_normalized = bestNormal.normalized();
 
        type = plane_jurdgment(bestNormal);

        if((type == 0) || (type == 3))  plane_types_data = ground;
        else if((type == 1) || (type == 4)) plane_types_data = y_wall;
        else if((type == 2) || (type == 5)) plane_types_data = x_wall;
        else plane_types_data = not_plane;

        planes.push_back(planeCloud);
        plane_types.push_back(plane_types_data);
        remainingCloud = cloudOutliers;

        end_normal_vector.push_back(bestNormal_normalized);
    }

    //for(int i = 1; i < 5; i++) ROS_INFO("%d   %f %f %f", cnt,abs(end_normal_vector[i][0]),abs(end_normal_vector[i][1]),abs(end_normal_vector[i][2]));     
    //최종 plane의 normal_vector에 abs 취한것

/* normal vector 간의 연관성있는 plane 추출*/
    int test = 6;
    bool start_num = ground_seg(end_normal_vector[0]);

    for(int k = start_num; k < 4; k++)
    {
        float theta;

        Eigen::Vector3f abs_vector_0(abs(end_normal_vector[start_num][0]),abs(end_normal_vector[start_num][1]),abs(end_normal_vector[start_num][2]));
        Eigen::Vector3f abs_vector_1(abs(end_normal_vector[k+1][0]),abs(end_normal_vector[k+1][1]),abs(end_normal_vector[k+1][2]));
    
        theta = acos(abs_vector_0.dot(abs_vector_1));

        if(theta < 5.0*M_PI/180) 
        {
            test = k;
            break;
        }
    }

    if(test != 6)ROS_INFO("%d plane %d plane is same axis", start_num, test+1);
    else ROS_INFO(" NOT SAME !!");

    return std::make_pair(plane_types, planes);

}

bool ground_seg(Eigen::Vector3f most_value_vector)
{
    Eigen::Vector3f abs_vector(abs(most_value_vector[0]), abs(most_value_vector[1]),abs(most_value_vector[2]));
    Eigen::Vector3f z_axis(0, 0, 1);

    int theta = acos(abs_vector.dot(z_axis));
    if(theta < 5.0*M_PI/180) 
    {
        return 1; // 바닥이라 판단  
    }

    return 0;
}

};




int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_converter_node");

  PointCloudConverter converter;

  ros::spin();

  return 0;
}
