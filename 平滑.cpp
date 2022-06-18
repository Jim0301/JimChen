#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>   //最小二乘平滑處理類定義
#include <pcl/visualization/pcl_visualizer.h>
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using namespace std;



void visualize_pcd(PointCloud::Ptr pcd_srcr)//, PointCloud::Ptr pcd_src)
{
	int vp_1;// , vp_2;
	// Create a PCLVisualizer object
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	viewer.createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	//viewer.createViewPort(0.5, 0, 1.0, 1.0, vp_2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_hr(pcd_srcr, 0, 255, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_hr(pcd_srcrr, 255, 0, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);

	viewer.addPointCloud(pcd_srcr, src_hr, "source cloudr", vp_1);
	//viewer.addPointCloud(pcd_srcrr, tgt_hr, "tgt cloudr",vp_1);
	//viewer.addPointCloud(pcd_src, src_h, "source cloud", vp_2);
	//viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud", vp_2);
	//viewer.addPointCloud(pcd_final, final_h, "final cloud", vp_2);*/
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}



int
main(int argc, char** argv)
 {
	// Load input file into a PointCloud<T> with an appropriate type
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Load bun0.pcd -- should be available with the PCL archive in test 
pcl::io::loadPCDFile("C:/Users/User/Desktop/TOOL/0318/filtest/12_48_icp05_nn01_sift_fil_test.pcd", *cloud);
	
// Create a KD-Tree
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
// Output has the PointNormal type in order to store the normals calculated by MLS
pcl::PointCloud<pcl::PointNormal> mls_points;
// Init object (second point type is for the normals, even if unused)
pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
// 設定在最小二乘計算中需要進行法線估計,不需要可跳過
mls.setComputeNormals(true);
 // Set parameters
mls.setInputCloud(cloud);
//mls.setPolynomialFit(false);  //多項式擬合提高精度,可false 加快速度,或選擇其他來控制平滑過程
mls.setSearchMethod(tree);
mls.setPolynomialOrder(3);
//mls.setSearchRadius(0.015);
mls.setSearchRadius(0.05);
//mls.setUpsamplingMethod(mls.RANDOM_UNIFORM_DENSITY);
//mls.setUpsamplingRadius(0.015);
//mls.setUpsamplingStepSize(0.01);
//mls.setPointDensity(20);
 // Reconstruct
//mls.setUpsamplingMethod(mls.VOXEL_GRID_DILATION);
//mls.setDilationVoxelSize(0.0001);
//mls.setDilationIterations(int iterations);
mls.process(mls_points);
// Save output
//pcl::io::savePCDFile("14-47_sift_mls_TEST.pcd", mls_points);
pcl::io::savePCDFile("C:/Users/User/Desktop/TOOL/0318/filtest/12_48_icp05_nn01_sift_fil_testmls015-2.pcd", mls_points);
}