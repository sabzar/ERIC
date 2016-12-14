
#ifndef EUCLIDEAN_H
#define EUCLIDEAN_H

#include <vector>

#include <rda\common\common.h>

namespace rda {

	void euclideanClusterExctraction(rda::cloudPtr cloud, std::vector<rda::cloudPtr>& clusters, double tolerance, int minSize, int maxSize);
}

#endif