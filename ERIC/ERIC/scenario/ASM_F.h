
#ifndef ASM_F_H
#define ASM_F_H

/*
 * 
 * Adaptive Split and Merge with new filter (reduce median filter)
 * 
 * 1. Filtration
 * 2. Clustering
 * 3. Monotone clustering
 * 4. Adaptive Rumer
 * 5. Least Squares Approximation
 * 6. Merging (not alway good for segment scanned objects)
 */

class ASM_F {

private:
	
public:
	/*ASM(double clustering_eps = 80, int clustering_minPts = 0, double min_rdp_eps = 15, double max_dist = 100,
		int min_part_size = 8, double merge_dist = 50, double merge_angle = 20, int filter_kN = 5,
		double filter_threashold = 0.7);*/

	void general(int argc, char* argv[]);

	void sectorScanning(int argc, char* argv[]);

	//void run();
};

#endif