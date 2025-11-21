#pragma once
#include "Tspline.h"
#include "SMath.hpp"
#include <CGAL/Polyhedron_items_3.h>
#include <CGAL/HalfedgeDS_default.h>
#include <CGAL/HalfedgeDS_vertex_base.h>
#include <CGAL/HalfedgeDS_halfedge_base.h>
#include <CGAL/HalfedgeDS_face_base.h>
#include <easy3d/core/surface_mesh.h>
#include <Eigen/Eigen>
#include <set>
#include <map>
#include <memory>

// Forward declaration
namespace tspline {
	class Tspline;
}

namespace skeleton {
	/**
	 * @brief extended vertex definition for skeleton polyhedron
	 *        stores control point, parametric coordinates, knot vectors, etc.
	 */
	template<class Refs, class Traits>
	struct skeleton_vertex : public CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, typename Traits::Point3d> {
	public:
		typedef typename Traits::Point3d Point3d;
		typedef typename Traits::Point4d Point4d;
		typedef typename Traits::Point2d Point2d;
		typedef CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, typename Traits::Point3d> Base;

		skeleton_vertex() : Base(), id(-1), patch_idx(-1), is_primary(false) {
			tsp.clear();
		}
		skeleton_vertex(const Point3d& p) : Base(p), id(-1), patch_idx(-1), is_primary(false) {}

		void SetCP(const Point4d& p) { cp = p; }
		const Point4d GetCP() const { return cp; }
		void set_param(const Point2d& p);


	public:
		tspline::Tspline tsp;
		int id;
		Point2d param;
		int patch_idx;
		std::vector<double> s;  // knot vector
		std::vector<double> t;  // knot vector
		bool is_primary;
	protected:
		Point4d cp;
	};

	/**
	 * @brief extended halfedge definition for skeleton polyhedron
	 *        stores geodesic distance, parametric distance, scaling factors, etc.
	 */
	template<class Refs, class Traits>
	struct skeleton_halfedge : public CGAL::HalfedgeDS_halfedge_base<Refs, CGAL::Tag_true> {
	public:
		typedef CGAL::HalfedgeDS_halfedge_base<Refs, CGAL::Tag_true> Base;
		typedef typename Traits::Point2d Point2d;  // 添加 Point2d 类型定义

		skeleton_halfedge() : Base(), d_geodestic(0.0), d_param(0.0), s(0.0) {
			t = Eigen::Vector2d(0.0, 0.0);
			param = Point2d();
			boundary_vertices.clear();
			f_id = 0;
			id = 0;
		}

		
		void set_d_gedestic(const double& d);
		void set_d_param(const double& d);


	public:
		int id;
		int f_id;
		double d_geodestic;  // geodesic distance of halfedge on the rootmesh
		double d_param;      // parametric distance of halfedge
		double s;            // the scaling factor between the dual halfedges (two different but adjacent local coordinate system)
		Eigen::Vector2d t;   // the translation between two different but adjacent local coordinate system
		Point2d param;       // the parametric coordinate of the halfedge
		std::map<int, int> boundary_vertices;  // the set of vertices on the rootmesh, set default counterclockwise
	};

	/**
	 * @brief extended face definition for skeleton polyhedron
	 *        stores error metrics, mesh patches, parameterization results, etc.
	 *        
	 * @note The third template parameter (Plane_3) is used to STORE the plane equation,
	 *       NOT to validate coplanarity. CGAL does NOT automatically verify that vertices
	 *       are coplanar. If vertices are not coplanar:
	 *       - The face can still be constructed (topologically valid)
	 *       - But the plane equation may be inaccurate or undefined
	 *       - Geometric operations may produce unexpected results
	 *       
	 *       It is the user's responsibility to ensure vertices are coplanar before
	 *       constructing faces, or to handle non-planar faces appropriately.
	 */
	template<class Refs, class Traits>
	struct skeleton_face : public CGAL::HalfedgeDS_face_base<Refs, CGAL::Tag_true, typename Traits::Plane_3> {
	public:
		typedef CGAL::HalfedgeDS_face_base<Refs, CGAL::Tag_true, typename Traits::Plane_3> Base_face;
		typedef typename Traits::Plane_3 Plane_3;

		skeleton_face() : Base_face(), error_sqr(0.0), min_point_count(0), id(0) {
			vertices_idx_map.clear();
		}

	public:
		int id;
		double error_sqr;
		unsigned min_point_count;

		std::shared_ptr<easy3d::SurfaceMesh> mesh;         // actually a patch of rootmesh
		std::shared_ptr<easy3d::SurfaceMesh> mesh_param;   // the patch's param result (parameterization result)
		std::shared_ptr<tspline::Tspline> mesh_tsp;         // the patch's TMesh which defined on the parametric
		std::map<int, int> vertices_idx_map;                // the mesh's resorted boundary vertices, which head (the idx=0) is the skeleton_vertex (must locate at left bottom)
		std::vector<int> corner_vertices;					// 
		std::vector<int> corner_skeleton_vertices;			//
	};

	/**
	 * @brief custom items for skeleton polyhedron
	 *        defines the vertex, halfedge, and face types used in the polyhedron
	 */
	struct SkeletonPolyhedron_items_3 : public CGAL::Polyhedron_items_3 {
		template<class Refs, class Traits>
		struct Vertex_wrapper {
			typedef typename Traits::Point3d Point3d;
			typedef skeleton_vertex<Refs, Traits> Vertex;
		};

		template<class Refs, class Traits>
		struct Halfedge_wrapper {
			typedef skeleton_halfedge<Refs, Traits> Halfedge;
		};

		template<class Refs, class Traits>
		struct Face_wrapper {
			typedef typename Traits::Plane_3 Plane;
			typedef skeleton_face<Refs, Traits> Face;
		};
	};

	/**
	 * @brief skeleton polyhedron type definition
	 *        uses Skeleton_Traits (which extends Traits_3) and SkeletonPolyhedron_items_3
	 *        Traits_3 ensures double precision support for all geometric operations
	 */
	typedef CGAL::Polyhedron_3<Skeleton_Traits, SkeletonPolyhedron_items_3> SkeletonPolyhedron;
}

// 包含模板实现
#include "TSkeletonTypes.inl"
