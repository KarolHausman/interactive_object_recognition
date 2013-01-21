/*
 * point_type.h
 *
 *  Created on: Jul 4, 2012
 *      Author: karol
 */

#ifndef POINT_TYPE_FH_
#define POINT_TYPE_FH_

#include <Eigen/Core>
#include <bitset>
#include <vector>
#include "pcl/ros/register_point_struct.h"
#include <pcl/point_types.h>

#include "dense_reconstruction/point_type.hpp"

namespace pcl {
	struct PointXYZRegionLF;
}
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZLRegionF,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, reg, reg)
                                   (uint32_t, label, label)
                                   (uint32_t, f, f)
);



#endif /* POINT_TYPE_H_ */
