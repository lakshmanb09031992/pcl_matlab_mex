// Standard libraries
#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include "mex.h"
// Point Cloud I/O related
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
// For PCL Filtering
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
// Outline removal
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
// For Ground Estimation
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
// Downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
// Kd Tree
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing.h>
// Bounding box
#include <boost/thread/thread.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>
// Kalman Filter


//---------------------------------------------------------------------------------------------------------------------------------------------------

/* GLOBAL VARIABLE DECLARATIONS */

// Structure for bounding box
struct measurement
{
	
	double x;
	double y;
	double z;
	double azimuth;
	uint32_t utc;
};



// Namespaces for PCL and OpenCV

using namespace std;

// Point Type (pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA)
typedef pcl::PointXYZI PointType;
// For Ground plane estimation
typedef pcl::PointCloud<PointType> PointCloudT;
// For clustering
typedef pcl::PointXYZINormal PointTypeFull;

//---------------------------------------------------------------------------------------------------------------------------------------------------

/* GLOBAL FUNCTION DECLARATIONS */

double distance_to_Line(pcl::PointXYZ line_start, pcl::PointXYZ line_end, pcl::PointXYZI point)
{
	double normalLength = _hypot(line_end.x - line_start.x, line_end.y - line_start.y);
	double distance = (double)((point.x - line_start.x) * (line_end.y - line_start.y) - (point.y - line_start.y) * (line_end.x - line_start.x)) / normalLength;
	return (distance);
}

double angle_to_line(pcl::PointXYZ line_start, pcl::PointXYZ line_end, pcl::PointXYZI point_intersect)
{
	double angle1 = atan2(line_start.y - point_intersect.y, line_start.x - point_intersect.x);
	double angle2 = atan2(line_end.y - point_intersect.y, line_end.x - point_intersect.x);
	double result = (angle2 - angle1) * 180 / 3.14;
	if (result < 0) {
		result += 360;
	}
	return result;
}

double angle_to_line(pcl::PointXYZ line_start, pcl::PointXYZ line_end, pcl::PointXYZ line2_start, pcl::PointXYZ line2_end)
{
	double angle1 = atan2(line_start.y - line_end.y, line_start.x - line_end.x);
	double angle2 = atan2(line2_start.y - line2_end.y, line2_start.x - line2_end.x);
	double result = (angle2 - angle1) * 180 / 3.14;
	if (result < 0) {
		result += 360;
	}
	return result;
}

uint32_t calculate_utc(uint32_t time,uint32_t tim_gap, float azimuth)
{
	uint32_t time_calcualte=0;
	uint32_t time_degree = 0;
	time_degree = tim_gap * (azimuth / 360);
	time_calcualte = time - time_degree;


	return time_calcualte;

}


//---------------------------------------------------------------------------------------------------------------------------------------------------
std::vector<std::vector<measurement>> matlabsend;

const char *fieldsPoint[] = {"x", "y", "z","azimuth","utc"};
/* MAIN FUNCTION */

int process_cloud(std::string pca)
{
	//-----------------------------------------------------------------------------------------------------------------------------------------------
    // Default values
	std::string ipaddress("192.168.1.70");
	std::string port("2368");
	std::string pcap;
    pcap=pca;
	std::string clbr("HDL-32.xml");
	/* VARIABLE DECLARATIONS */
    std::cout << "-ipadress : " << ipaddress << std::endl;
	std::cout << "-port : " << port << std::endl;
	std::cout << "-pcap : " << pcap << std::endl;
	std::cout << "-clbr : " << clbr << std::endl;
	
	std::vector<measurement> meas_snd(10);


// Clock
	clock_t tStart;
	double delta_t = 0.1;

	// Trial parameters
	bool downsample = false,
		eucledian = true,
		region_growing = false,
		bounding = true,
		bounding_alt = false,
		debug = false;

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	// For Filtering
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);

	// For ground plane estimation 
	PointCloudT::Ptr	cloud_inliers(new PointCloudT), cloud_outliers(new PointCloudT);

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr 		inliers_plane(new pcl::PointIndices);
	PointCloudT::Ptr 			cloud_plane(new PointCloudT);
	pcl::ExtractIndices<PointType> extract;
	pcl::SACSegmentation<PointType> seg;				// Create the segmentation object
	plane->values.resize(4);                            // 4 points to form a plane

	// Search tree 
	pcl::search::KdTree<PointType>::Ptr search_tree(new pcl::search::KdTree<PointType>);

	//-----------------------------------------------------------------------------------------------------------------------------------------------

	/* OUTPUT VIEWER WINDOW SETUP */

	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Velodyne Viewer"));
	viewer->addCoordinateSystem(3.0, "coordinate");
	viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);

	// Point Cloud Color handler
	pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;

	const std::type_info& type = typeid(PointType);
	if (type == typeid(pcl::PointXYZ)) {
		std::vector<double> color = { 0, 255.0, 0 };
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(color[0], color[1], color[2]));
		handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZI)) {
		std::vector<double> color = { 255.0, 255.0, 255.0 };
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(color[0], color[1], color[2]));
		handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZRGBA)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<PointType>());
		handler = color_handler;
	}
	else {
		throw std::runtime_error("This PointType is unsupported.");
	}

	//-----------------------------------------------------------------------------------------------------------------------------------------------

	/* POINT CLOUD EXTRACTION FROM PCAP FILE */

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);

		// Point Cloud Processing
		cloud = ptr;
	};


	// HDL Grabber
	boost::shared_ptr<pcl::HDLGrabber> grabber;
	if (!pcap.empty()) {
		std::cout << "Capture from PCAP..." << std::endl;
		grabber = boost::shared_ptr<pcl::HDLGrabber>(new pcl::HDLGrabber(clbr, pcap));
	}
	else if (!ipaddress.empty() && !port.empty()) {
		std::cout << "Capture from Sensor..." << std::endl;
		grabber = boost::shared_ptr<pcl::HDLGrabber>(new pcl::HDLGrabber(boost::asio::ip::address::from_string(ipaddress), boost::lexical_cast<unsigned short>(port)));

	}
	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);
	
	// Start Grabber
	grabber->start();

	//-----------------------------------------------------------------------------------------------------------------------------------------------
	const float fps = grabber->getFramesPerSecond();

	/* CORE PROCESSING */
	bool first_frame = false; uint64_t time_frame; float current_time = 0; uint32_t last_time = 0, ls_t = 0; uint32_t n, u;

	while (!viewer->wasStopped())
	{
		bool send_measurement = false;
		viewer->spinOnce(); // Update Viewer
		tStart = clock();
		boost::mutex::scoped_try_lock lock(mutex);
		unsigned int lastangle=grabber->last_azimuth_;
		if (lock.owns_lock() && cloud) 
		{
			if (debug)
			{
				std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
					<< " data points (" << pcl::getFieldsList(*cloud) << ").\n\n";
			}

			/*Get time stamp data*/
			uint32_t time_gap = 0;
			time_frame = cloud->header.stamp;
			n = static_cast<uint32_t>(time_frame);
			u = time_frame >> 32;
			cout << (n - last_time) << endl;
			time_gap = n - last_time;
			last_time = n;
			std::string frameid=cloud->header.frame_id;
			cout << frameid << endl;
			//---------------------------------------------------------------------------------------------------------------------------------------
		

			/* FILTERING THE Z AXIS AND DOWNSAMPLING */

		
			if (downsample)
			{
				// Downsample the cloud and Create the filtering object
				pcl::VoxelGrid<PointType> downsample;
				downsample.setInputCloud(cloud_filtered);
				downsample.setLeafSize(0.07f, 0.07f, 0.0f);
				downsample.setDownsampleAllData(true);
				downsample.filter(*cloud_filtered);
			}

			// Handler for color selection of  Point Cloud
			handler->setInputCloud(cloud);

			pcl::PassThrough<PointType> pass_z;
			pass_z.setInputCloud(cloud);
			pass_z.setFilterFieldName("z");
			pass_z.setFilterLimits(-2.0f, 10.0f);
			pass_z.setFilterLimitsNegative(false);
			pass_z.filter(*cloud_outliers);

		
			pcl::PassThrough<PointType> pass;
			pass.setInputCloud(cloud_outliers);
			pass.setFilterFieldName("intensity");
			pass.setFilterLimits(150.0f, 300.0f);
			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_outliers);
			
			if (debug)
			{
				std::cerr << "Stat Outliners Final: " << cloud_outliers->width * cloud_outliers->height
					<< " data points (" << pcl::getFieldsList(*cloud_outliers) << ").\n\n";
			}
			if ((!viewer->updatePointCloud(cloud_outliers, *handler, "cloud outs")))
			{
				viewer->addPointCloud(cloud_outliers, *handler, "cloud outs");

			}
			
			if (0)
			{
				pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb2(cloud_inliers, 255.0, 0.0, 0.0); //This will display the point cloud in green (R,G,B)
				if ((!viewer->updatePointCloud(cloud_inliers, rgb2, "cloud ins")))
				{
					viewer->addPointCloud(cloud_inliers, rgb2, "cloud ins");

				}
			}


			//---------------------------------------------------------------------------------------------------------------------------------------

			/* CLUSTERING */

			search_tree->setInputCloud(cloud_outliers);			// Kd Tree Data Structure
			std::vector<pcl::PointIndices> cluster_indices;		// Extraxt indices here

			// Cluster algorithm

			// Eucledian 
			if (eucledian)
			{
				// Eucledian
				pcl::EuclideanClusterExtraction<PointType> ec;
				ec.setClusterTolerance(0.8); // 2cm
				ec.setMinClusterSize(3);
				ec.setMaxClusterSize(6000);
				ec.setSearchMethod(search_tree);
				ec.setInputCloud(cloud_outliers);
				ec.extract(cluster_indices);
			}

			//---------------------------------------------------------------------------------------------------------------------------------------

			/* BOUNDING BOX */

			if (bounding)
			{
				viewer->removeAllShapes();

				// Extract the indices of the cloud
				std::vector<pcl::PointCloud<PointType>::Ptr> cloud_cluster_vector;// (new pcl::PointCloud<PointType>);
				int j = 0;

				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
				{

					pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
					for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
						cloud_cluster->points.push_back(cloud_outliers->points[*pit]); //*
					cloud_cluster->width = cloud_cluster->points.size();
					cloud_cluster->height = 1;
					cloud_cluster->is_dense = true;
					if (debug)
					{
						std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
					}
					cloud_cluster_vector.push_back(cloud_cluster);
					j++;
				}

				// Bounding box creation
				for (int i = 0; i < cloud_cluster_vector.size(); i++)
				{

					/*+++++++++++++++
					Outliner removal in each cluster
					+++++++++++++++++*/
				
					std::stringstream ss1, cluster,ss2;
				
					/**+++++++++++++++
					Bounidng box generation
					++++++++++++++++++*/

					pcl::MomentOfInertiaEstimation <PointType> feature_extractor;
					feature_extractor.setInputCloud(cloud_cluster_vector[i]);
					feature_extractor.compute();

					pcl::PointXYZI min_point_AABB;
					pcl::PointXYZI max_point_AABB;
					feature_extractor.getAABB(min_point_AABB, max_point_AABB);
					ss1 << "line_1" << i;
					ss2 << "s2" << i;

					float cluster_x = 0, cluster_y = 0, cluster_angle = 0;
					uint32_t time_cluster = 0;
					cluster_x = (max_point_AABB.x + min_point_AABB.x) / 2;
					cluster_y = (max_point_AABB.y + min_point_AABB.y) / 2;
					float distance;
					distance = sqrt(pow(cluster_x, 2.0) + pow(cluster_y, 2.0));
					if (distance < 0.8)
					{
						continue;
					}
					cluster_angle = std::atan2(cluster_y,cluster_x);
					meas_snd[i].x = cluster_x*cos(-3.14159)+cluster_y*sin(-3.14159);
					meas_snd[i].y = cluster_y*cos(-3.14159)-cluster_x*sin(-3.14159);
					if (cluster_angle < 0)
					{
						cluster_angle = 360 - cluster_angle;
					}
					meas_snd[i].azimuth = cluster_angle;
					time_cluster = calculate_utc(n,time_gap, meas_snd[i].azimuth);

					meas_snd[i].utc = time_cluster;
					
					send_measurement = true;
					viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, ss1.str());
					viewer->setRepresentationToWireframeForAllActors();

				}
			}
		}
		if(send_measurement)
		{ 

			matlabsend.push_back(meas_snd);

		}
	}

	//-----------------------------------------------------------------------------------------------------------------------------------------------

	/* WINDING THINGS DOWN */

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;
}




const char *fieldsStruct[] = {"cloud"};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    mxArray *p;
    // convert it to MATLAB struct array and return it as output
     std::string input_buf=mxArrayToString(prhs[0]);
    process_cloud(input_buf);
   
    plhs[0] = mxCreateStructMatrix(1,matlabsend.size() , 1, fieldsStruct);
    for (int i=0; i<matlabsend.size(); i++) 
    {
         p = mxCreateStructMatrix(matlabsend[i].size(), 1, 5, fieldsPoint);
        // start point
        for(int j=0;j<matlabsend[j].size();j++)
        {
        mxSetField(p, j, "x", mxCreateDoubleScalar(matlabsend[i][j].x));
        mxSetField(p, j, "y", mxCreateDoubleScalar(matlabsend[i][j].y));
		mxSetField(p, j, "z", mxCreateDoubleScalar(matlabsend[i][j].z));
		mxSetField(p, j, "azimuth", mxCreateDoubleScalar(matlabsend[i][j].azimuth));
		mxSetField(p, j, "utc", mxCreateDoubleScalar(matlabsend[i][j].utc));
        }
          mxSetField(plhs[0], i, "cloud", p);
        
        
    }
}