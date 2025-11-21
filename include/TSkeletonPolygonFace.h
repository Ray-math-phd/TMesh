#pragma once
#include "TSkeleton.h"

namespace skeleton {
	/**
	 * Base class, offer the return functions.
	 * .
	 */
	class BasePolygonModifier :public CGAL::Modifier_base<TSkeleton::HalfedgeDS> {
	protected:
		TSkeleton::Halfedge_handle first_border_halfedge_;
	public:
		TSkeleton::Halfedge_handle get_first_border_halfedge()const;
	};

	/**
	 * extended class, construct the simplePolygon.
	 * .
	 */
	class SimplePolygonModifier :public BasePolygonModifier {
	private:
		const std::vector<Point3d>& points_;
		const TSkeleton& skt_;
		TSkeleton::Halfedge_handle& he_;// the opposite halfedge.
	public:
		SimplePolygonModifier(const std::vector<Point3d>& points,
			const TSkeleton& skt,
			TSkeleton::Halfedge_handle& he) :points_(points),skt_(skt) , he_(he) {}
		void operator()(TSkeleton::HalfedgeDS& hds)override;
		void judge_mode(int& mode);

		//==================================== auxiliary functions =========================================
		void judge_polygon_vertices_ccw_existed(std::vector<Point3d>& points__, std::vector<bool>& is_existed);

		TSkeleton::Halfedge_handle get_exsited_half_edge_ccw(Point3d& s_pos, Point3d& t_pos);

	private:
		void create_simple_polygon(TSkeleton::HalfedgeDS& hds);
		void add_simple_polygon(TSkeleton::HalfedgeDS& hds);
	};

	//==================================================== halfedge modifier ========================================

	/**
	 * 
	 * .
	 */
}
