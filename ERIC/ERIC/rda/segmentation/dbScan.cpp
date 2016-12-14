
#include <string>
#include <vector>
//#include <algorithm>
#include <math.h>
#include <time.h>
#include <iostream>

#include "dbScan.h"

using namespace rda;

DbScan::DbScan(){
}

DbScan::DbScan(Net* n) : net(n) { 

	// intit points clusters -1 (noise) 
	for(rda::Matrix<rda::Cell>::iterator it = n->matrix().begin(); it != n->matrix().end(); ++it){
		for( int i=0; i < (*it).getPoints().size(); i++ ){
			(*it).getPoint(i)->z = -1;
		}
	}
}

void DbScan::absorbCluster(int absorbent, int absorbed){			

	for (int u = 0; u < (int)clusters.at(absorbed).size(); u++){
		clusters[absorbed][u]->z = absorbent;
			//clusterInd[clusters[absorbed][u]] = absorbent;
			//(*clusters[absorbed][u]).cluster = absorbent;
		}
		
		for (int u = 0; u < (int)clusters[absorbed].size(); u++){
			clusters[absorbent].push_back(clusters[absorbed][u]);
		}
		
		clusters[absorbed].clear();		
	}

void DbScan::setPointCluster(pcl::PointXYZ* p, int offset){
		p->z = offset;
		//clusterInd[p] = offset;
		//(*p).cluster = offset;
		clusters[offset].push_back(p);
	}

double DbScan::getDistance(pcl::PointXYZ& p1, pcl::PointXYZ& p2){
		double d = sqrt(powf((p2.x - p1.x), 2.0f) + powf((p2.y - p1.y), 2.0f));
		return d;
	}

bool DbScan::distance(pcl::PointXYZ& p1, pcl::PointXYZ& p2){
		if (getDistance(p1, p2) <= (*net).cellSize())
			return true;
		return false;
	}

void DbScan::scanAreaInNet(int iFrom, int jFrom, int iTo, int jTo, int diFrom, int djFrom, int diTo, int djTo, int minPts){

	std::vector<pcl::PointXYZ*> pointsToAdd;

		for (int i = iFrom; i < iTo; i++){
			for (int j = jFrom; j < jTo; j++){
				for (int l = 0; l < (*net)[i][j].getPoints().size(); l++){

					pcl::PointXYZ* currPoint = (*net)[i][j].getPoint(l);

					for (int k = i - diFrom; k < i + diTo; k++){
						for (int m = j - djFrom; m < j + djTo; m++){
													
							for (int c = 0; c < (*net)[k][m].getPoints().size(); c++){
							
								pcl::PointXYZ* point = (*net)[k][m].getPoint(c);
								if(c == l && ((i == k) && (j == m))){}
								else{
									if (distance((*currPoint), (*point))){
										pointsToAdd.push_back(point);
									}
								}
							}
						}
					}
					expandCluster(pointsToAdd, currPoint, minPts);
				}//l end
			}
		}
	}

void DbScan::expandCluster(std::vector<pcl::PointXYZ*>& neighbours, pcl::PointXYZ* currPoint, int minPts){

		if ((int)neighbours.size() >= minPts){
			if (currPoint->z < 0){				
				clusters.push_back(Cluster());
				setPointCluster(currPoint, clusters.size() - 1);
			}
			for (int z = 0; z < (int)neighbours.size(); z++){
				pcl::PointXYZ* point = neighbours[z];
				if (point->z < 0){
					setPointCluster(point, currPoint->z);
				}
				else
					if (point->z != currPoint->z){
						if (clusters[(int)point->z].size() < clusters[(int)currPoint->z].size()){
							absorbCluster(currPoint->z, point->z);
					}
					else{
						absorbCluster(point->z, currPoint->z);
					}
				}
			}
		}
		neighbours.clear();
	}




void DbScan::dbScan(int minPts, double cellWidth, bool computeMinPts){

		int N = (*net).matrix().columnsNumber();//getWidth() / net.getCellWidth();
		int M = (*net).matrix().rowsNumber();//getHeight() / net.getCellHeight();		

		// middle
		scanAreaInNet(1, 1, M - 1, N - 1, 1, 1, 2, 2, minPts);

		//left border		
		scanAreaInNet(1, 0, M - 1, 1, 1, 0, 2, 2, minPts);

		// right border
		scanAreaInNet(1, N - 1, M - 1, N, 1, 1, 2, 1, minPts);

		//top border
		scanAreaInNet(0, 1, 1, N - 1, 0, 1, 2, 2, minPts);

		//bottom border
		scanAreaInNet(M - 1, 1, M, N - 1, 1, 1, 1, 2, minPts);


		//leftTop cell
		scanAreaInNet(0, 0, 1, 1, 0, 0, 2, 2, minPts);


		//rightTop cell
		scanAreaInNet(0, N - 1, 1, N, 0, 1, 2, 1, minPts);

		//leftBottom cell
		scanAreaInNet(M - 1, 0, M, 1, 1, 0, 1, 2, minPts);

		//rightBottom cell
		scanAreaInNet(M - 1, N - 1, M, N, 1, 1, 1, 1, minPts);

		//get z coordinate to 1 again
		for(int i=0 ; i < clusters.size(); i++){
			for(int j = 0; j < clusters[i].size(); j++){
				clusters[i][j]->z = 0;
			}
		}
	}

void DbScan::removeEmptyClusters(){	
		
	for(std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end();/*it++ - error*/){
		if((*it).size() < 1)
			it =  clusters.erase(it);
		else
			++it;
	}
}

#pragma region GettersSetters

	Net& DbScan::getNet(){
		return (*net);
	}

	std::vector<Cluster>& DbScan::getClusters(){
		return clusters;
	}

#pragma endregion

DbScan::~DbScan()
	{
		clusters.clear();
	}



void rda::dbScan(rda::cloudPtr input_cloud, double eps, int minPts, std::vector<rda::cloudPtr>& clusters){

	clock_t filter_clock;
	filter_clock = clock();

	rda::Net net(input_cloud, eps);
	rda::DbScan seg(&net);
	seg.dbScan(minPts, eps);
	seg.removeEmptyClusters();

	std::cout << "Segmentation only :" <<  ((float)(clock() - filter_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;

	std::vector<rda::Cluster> cls = seg.getClusters();
	
	for(int i = 0; i < cls.size(); i++){

		rda::cloudPtr ptr (new pcl::PointCloud<pcl::PointXYZ>);
		clusters.push_back( ptr ) ;

		for(int j = 0; j < cls[i].size(); j++){
			clusters[i]->push_back( *cls[i][j] ) ;
		}
	}

}