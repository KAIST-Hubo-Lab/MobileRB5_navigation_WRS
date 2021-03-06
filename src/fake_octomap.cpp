#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap_msgs/conversions.h>

/*
 * Define an axis-aligned bounding box of obstacle and then update the map to represent the obstacle space
 *
 * _center: the center position of AABB
 * _length: the length of AABB for each axis
 * _octree: the map to represent the given AABB
 */

ros::Publisher octomap_publisher;

void update_obstacle(const octomap::point3d& _center, const octomap::point3d& _length, octomap::OcTree& _octree);

void pc2_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Generate a fake octomap for test
    const double RESOLUTION = 0.05;
    octomap::OcTree octree(RESOLUTION);
    for(int i = 0; i < cloud->size(); i++){
        // Add obstacles to octree
                // update_obstacle(center point, length of each edge, octree)
        //if (cloud->at(i).z > 0.4)
			update_obstacle(octomap::point3d(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z), octomap::point3d(RESOLUTION,RESOLUTION,RESOLUTION), octree);
    }

    // Publish the generated map
    octomap_msgs::Octomap fake_map_msg;
    fake_map_msg.header.frame_id = "map";
    fake_map_msg.header.stamp = ros::Time::now();
    octomap_msgs::binaryMapToMsg(octree, fake_map_msg);

    octomap_publisher.publish(fake_map_msg);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "fake_octomap");

    ros::NodeHandle nh;
    // Subscriber
    ros::Subscriber pc2_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_matched_points2", 10, pc2_callback);

    // Publisher
    octomap_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/fake_map", 1);

	ros::Rate r(10);
	
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
    

    return 0;
}

void update_obstacle(const octomap::point3d& _center, const octomap::point3d& _length, octomap::OcTree& _octree)
{
    // Append voxels to the octree
    octomap::OcTreeKey minKey = _octree.coordToKey(_center - (_length * 0.5));
    octomap::OcTreeKey maxKey = _octree.coordToKey(_center + (_length * 0.5));
    for(uint16_t kx = minKey[0]; kx <= maxKey[0]; kx++){
        for(uint16_t ky = minKey[1]; ky <= maxKey[1]; ky++){
            for(uint16_t kz = minKey[2]; kz <= maxKey[2]; kz++){
                _octree.updateNode(octomap::OcTreeKey(kx, ky, kz), true);
            }
        }
    }
}
