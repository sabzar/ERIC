
#include <pcl\kdtree\kdtree.h>
#include <pcl\segmentation\extract_clusters.h>

#include <rda\segmentation\euclidean.h>
		  
void rda::euclideanClusterExctraction(rda::cloudPtr cloud, std::vector<rda::cloudPtr>& clusters, double tolerance, int minSize, int maxSize){

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ> );
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (tolerance); 
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){

		rda::cloudPtr cluster (new pcl::PointCloud<pcl::PointXYZ>);

		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			cluster->push_back(cloud->points[*pit]);
		}

		clusters.push_back(cluster);
	}
}