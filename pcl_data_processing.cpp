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

// Namespaces for PCL and OpenCV

using namespace std;

// Point Type (pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA)
typedef pcl::PointXYZI PointType;
// For Ground plane estimation
typedef pcl::PointCloud<PointType> PointCloudT;
// For clustering
typedef pcl::PointXYZINormal PointTypeFull;


// For Filtering
pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
// Point Cloud
pcl::PointCloud<PointType>::ConstPtr cloud;
// For ground plane estimation 
PointCloudT::Ptr	cloud_inliers(new PointCloudT), cloud_outliers(new PointCloudT);

// Segment the ground
pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr 		inliers_plane(new pcl::PointIndices);
PointCloudT::Ptr 			cloud_plane(new PointCloudT);
struct measurement
{

	double x;
	double y;
	double z;
	double azimuth;
	uint32_t utc;

};

//---------------------------------------------------------------------------------------------------------------------------------------------------

/* GLOBAL VARIABLE DECLARATIONS */

std::vector<measurement> meas_snd(10);
std::vector<std::vector<measurement>> matlabsend;
bool first_frame = false; uint64_t time_frame; float current_time = 0; uint32_t last_time = 0, ls_t = 0; uint32_t n, u;
bool send_measurement = false;
// Structure for bounding box





//---------------------------------------------------------------------------------------------------------------------------------------------------

/* GLOBAL FUNCTION DECLARATIONS */
void vector_clear()
{
	
	for (int i = 0; i < 10; i++)
	{
		meas_snd[i].x = 0; meas_snd[i].y = 0; meas_snd[i].z = 0; meas_snd[i].utc = 0; meas_snd[i].azimuth=0;
	}
}

uint32_t calculate_utc(uint32_t time, uint32_t tim_gap, float azimuth)
{
	uint32_t time_calcualte = 0;
	uint32_t time_degree = 0;
	time_degree = tim_gap * ((6.28319-azimuth) / 6.28319);
	time_calcualte = time - time_degree;


	return time_calcualte;

}

void pass_inte(pcl::PointCloud<PointType>::ConstPtr ptr)
{
	pcl::PassThrough<PointType> pass;
	pass.setInputCloud(ptr);
	pass.setFilterFieldName("intensity");
	pass.setFilterLimits(150.0f, 300.0f);
	pass.setFilterLimitsNegative(false);
	pass.filter(*cloud_outliers);
	pass.setFilterLimitsNegative(true);
	pass.filter(*cloud_inliers);
}

void cluster(uint32_t n)
{
	/* CLUSTERING */
    uint32_t time_s=0;
    time_s = n - last_time;
    last_time=n;
				// Search tree 
	pcl::search::KdTree<PointType>::Ptr search_tree(new pcl::search::KdTree<PointType>);
	search_tree->setInputCloud(cloud_outliers);			// Kd Tree Data Structure
	std::vector<pcl::PointIndices> cluster_indices;		// Extraxt indices here
    // Extract the indices of the cloud
        std::vector<pcl::PointCloud<PointType>::Ptr> cloud_cluster_vector;
	// Cluster algorithm

	// Eucledian 
	if (1)
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

	if (1)
	{
		

		// (new pcl::PointCloud<PointType>);
		int j = 0;

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{

			pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				cloud_cluster->points.push_back(cloud_outliers->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			
			cloud_cluster_vector.push_back(cloud_cluster);
			j++;
		}

		// Bounding box creation
		for (int i = 0; i < cloud_cluster_vector.size(); i++)
		{

			/*+++++++++++++++
			Outliner removal in each cluster
			+++++++++++++++++*/

			std::stringstream ss1, cluster, ss2;

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
			
			meas_snd[i].x = cluster_x * cos(-3.14159) + cluster_y * sin(-3.14159);
			meas_snd[i].y = cluster_y * cos(-3.14159) - cluster_x * sin(-3.14159);
            cluster_angle = std::atan2(cluster_x, cluster_y);
			if (cluster_angle < 0.0) 
            {
                cluster_angle += 6.28319;
            }
            meas_snd[i].azimuth = n;
            uint32_t ut=calculate_utc(n,time_s, cluster_angle);
            meas_snd[i].utc = ut;
			
			//time_cluster = calculate_utc(n, time_gap, meas_snd[i].azimuth);
			send_measurement = true;
		}
	}
}
void vector_send(bool send)
{
	if(send)
		matlabsend.push_back(meas_snd);	
}



const char *fieldsPoint[] = { "x", "y", "z","azimuth","utc" };
/* MAIN FUNCTION */

int process_cloud(std::string pca)
{
	//-----------------------------------------------------------------------------------------------------------------------------------------------
	// Default values
	std::string ipaddress("192.168.1.70");
	std::string port("2368");
	std::string pcap;
	pcap = pca;
	std::string clbr("HDL-32.xml");
	/* VARIABLE DECLARATIONS */
	std::cout << "-ipadress : " << ipaddress << std::endl;
	std::cout << "-port : " << port << std::endl;
	std::cout << "-pcap : " << pcap << std::endl;
	std::cout << "-clbr : " << clbr << std::endl;
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
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =[&mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) 
    {
		boost::mutex::scoped_lock lock(mutex);
		uint64_t time_frame = ptr->header.stamp;
		uint32_t n_t = static_cast<uint32_t>(time_frame);
		cout << n_t << endl;
		// Point Cloud Processing
		vector_clear();
		pass_inte(ptr);
		cluster(n_t);
		vector_send(send_measurement);
		
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


	while (!viewer->wasStopped())
	{
		
		viewer->spinOnce(); // Update Viewer
		
		boost::mutex::scoped_try_lock lock(mutex);
		unsigned int lastangle = grabber->last_azimuth_;
		if (lock.owns_lock() && cloud_outliers)
		{
			// Handler for color selection of  Point Cloud
			handler->setInputCloud(cloud_outliers);
            viewer->removeAllShapes();
			if ((!viewer->updatePointCloud(cloud_outliers, *handler, "cloud outs")))
			{
				viewer->addPointCloud(cloud_outliers, *handler, "cloud outs");

			}

			if (1)
			{
				pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb2(cloud_inliers, 255.0, 0.0, 0.0); //This will display the point cloud in green (R,G,B)
				if ((!viewer->updatePointCloud(cloud_inliers, rgb2, "cloud ins")))
				{
					viewer->addPointCloud(cloud_inliers, rgb2, "cloud ins");

				}
			}
			//--------------------------------------------------------------------------------------------------------------------------------------
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




const char *fieldsStruct[] = { "cloud" };

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	mxArray *p;
	// convert it to MATLAB struct array and return it as output
	std::string input_buf = mxArrayToString(prhs[0]);
	process_cloud(input_buf);

	plhs[0] = mxCreateStructMatrix(1, matlabsend.size(), 1, fieldsStruct);
	for (int i = 0; i < matlabsend.size(); i++)
	{
		p = mxCreateStructMatrix(matlabsend[i].size(), 1, 5, fieldsPoint);
		// start point
		for (int j = 0; j < matlabsend[j].size(); j++)
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