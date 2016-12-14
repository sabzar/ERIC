/*
 * DbScan.h
*/

#ifndef DbScan_H
#define DbScan_H

#include <unordered_map>
#include <vector>

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

#include <rda\search\grid.h>
#include <rda\common.h>


namespace rda {

	typedef std::vector<pcl::PointXYZ*> Cluster;	

	class DbScan
	{
	public:

		DbScan();

		DbScan(Net* net);

		void dbScan(int minPts, double cellWidth, bool computeMints = false);

		void removeEmptyClusters();

		//std::vector<Point>& getPoints();

		Net& getNet();

		std::vector<Cluster>& getClusters();

		~DbScan();


	private:

		void setPointCluster(pcl::PointXYZ* p, int offset);

		void absorbCluster(int absorbent, int absorbed);

		//metric
		bool distance(pcl::PointXYZ& p1, pcl::PointXYZ& p2);
		
		double getDistance(pcl::PointXYZ& p1, pcl::PointXYZ& p2);
		// scan local area (9 cells)
		void scanAreaInNet(int iFrom, int jFrom, int iTo, int jTo, int diFrom, int djFrom, int diTo, int djTo, int minPts);
		
		void expandCluster(std::vector<pcl::PointXYZ*>& neighbours, pcl::PointXYZ* currPoint, int minPts);

		Net* net;
		std::vector<Cluster> clusters;

	};

	void dbScan(rda::cloudPtr input_cloud, double eps, int minPts, std::vector<rda::cloudPtr>& clusters);
}

#endif
