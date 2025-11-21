#pragma once
#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>

namespace skeleton {
	// Kernel definition using double precision
	typedef CGAL::Cartesian<double> Kernel;
	
	/**
	 * @brief Custom Traits_3 for Polyhedron_3
	 *        Explicitly defines all required types for 3D polyhedron operations
	 *        Uses double precision arithmetic
	 *        
	 *        This traits class provides all necessary type definitions for CGAL::Polyhedron_3
	 *        and ensures double precision support for all geometric operations.
	 */
	struct Traits_3 {
		// Required types for Polyhedron_3 (CGAL convention)
		typedef Kernel::Point_3 Point_3;
		typedef Kernel::Plane_3 Plane_3;
		
		// Additional useful geometric types
		typedef Kernel::Vector_3 Vector_3;
		typedef Kernel::Segment_3 Segment_3;
		typedef Kernel::Ray_3 Ray_3;
		typedef Kernel::Line_3 Line_3;
		typedef Kernel::Triangle_3 Triangle_3;
		
		// Convenience typedefs (common naming)
		typedef Kernel::Point_2 Point2d;
		typedef Kernel::Point_3 Point3d;
		typedef Kernel::Vector_2 Vector2d;
		typedef Kernel::Vector_3 Vector3d;
		typedef Kernel::Plane_3 Plane3d;
		
		// Constructors
		Traits_3() {}
		Traits_3(const Traits_3&) {}
	};
	
	// Convenience typedefs for direct access
	typedef Traits_3::Point2d Point2d;
	typedef Traits_3::Point3d Point3d;
	typedef Traits_3::Vector2d Vector2d;
	typedef Traits_3::Vector3d Vector3d;
	typedef Traits_3::Plane3d Plane_3;

	/**
	 * @brief extends Point3d by the weight entry for control points (weight is 1.0 by default)
	 *        similar to tspline::Point4d
	 */
	class Point4d : public Point3d {
	protected:
		double weight;

	public:
		Point4d() : Point3d(), weight(1.0) {}
		Point4d(const Point3d& a, const double& w = 1.0) : Point3d(a), weight(w) {}
		Point4d(const double& x, const double& y, const double& z, const double& w) :
			Point3d(x, y, z), weight(w) {}
		Point4d(const double& x, const double& y, const double& z) :
			Point3d(x, y, z), weight(1.0) {}

		void operator=(const Point3d& a) {
			*this = Point4d(a);
		}

		double w() const { return weight; }
		void set_weight(double w) { weight = w; }
	};

	/**
	 * @brief unified traits structure for skeleton types
	 *        extends Traits_3 with additional types like Point4d
	 *        provides all necessary type definitions for skeleton operations
	 */
	struct Skeleton_Traits {
		// Inherit all types from Traits_3
		typedef Traits_3::Point_3 Point_3;
		typedef Traits_3::Plane_3 Plane_3;
		typedef Traits_3::Vector_3 Vector_3;
		typedef Traits_3::Segment_3 Segment_3;
		typedef Traits_3::Ray_3 Ray_3;
		typedef Traits_3::Line_3 Line_3;
		typedef Traits_3::Triangle_3 Triangle_3;
		
		// Convenience typedefs
		typedef Traits_3::Point2d Point2d;
		typedef Traits_3::Point3d Point3d;
		typedef Traits_3::Vector2d Vector2d;
		typedef Traits_3::Vector3d Vector3d;
		
		// Extended types
		typedef Point4d Point4d;  // Extended type with weight
		typedef Vector2d Vector_2;  // Alias for Vector2d
	};
}
