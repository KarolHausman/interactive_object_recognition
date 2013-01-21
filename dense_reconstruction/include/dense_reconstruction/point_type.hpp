/*
 * point_type.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: karol
 */

#ifndef POINT_TYPE_FHPP_
#define POINT_TYPE_FHPP_

namespace pcl {

	struct PointXYZLRegionF
	{
			PCL_ADD_POINT4D;
			float reg;
			uint32_t label;
			uint32_t f;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	}EIGEN_ALIGN16;
	inline std::ostream& operator << (std::ostream& os, const PointXYZLRegionF& p)
	{
	  os << "(" << p.x << "," << p.y << "," << p.z << "," << p.reg << "," << p.label <<","<<p.f <<")";
	  return (os);
	}



}


#endif /* POINT_TYPE_HPP_ */
