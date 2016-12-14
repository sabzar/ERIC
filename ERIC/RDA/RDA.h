
/*
 * RDA.h
 * Rangefiner data analysis
 *
*/

#ifndef RDA_H
#define RDA_H

extern "C" {
	
	/*
	 * 
	 *	-clustering_eps 80 
	 * 	-clustering_minPts 0 
	 * 	-min_rdp_eps 15 
	 * 	-max_dist 100 
	 * 	-min_part_size 8 
	 * 	-merge_dist 50 | if sector -> -10
	 * 	-merge_angle 20 
	 * 	-filter_kN 5 
	 * 	-filter_threshold 0.9 | if model data -> 5.0
	*/

	__declspec(dllexport) void __cdecl extractLines(double* input, double clustering_eps, int clustering_minPts, double min_rdp_eps, double max_dist, int min_part_size, double merge_dist, double merge_angle, int filter_kN, double filter_treshold, int sensor_id, double**& output, int& clusters_size);

	// Returns the number of elements needed to store point in array
	__declspec(dllexport) int __cdecl pointSize();

	//__declspec(dllexport) void __cdecl clearMemory(double*& ptr);

	__declspec(dllexport) void __cdecl clearMemory(double**& ptr, int size);
}
	
#endif