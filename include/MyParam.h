#pragma once
#pragma once
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/types.h>

#include <map>
#include <set>
#include <iterator>

#include "Tspline.h"

namespace myparam {

	class MyParam {
	public:
		MyParam(const easy3d::SurfaceMesh* orignal_mesh_);
		/**
		 * The default parameter passed to the constructor, std::vector<int>& corner_verices_, 
		 * must have its first element as the origin of the parameter domain and be arranged 
		 * in counterclockwise order.
		 * 
		 * The first elements of the default parameters std::vector<int>& boundary_vertices_ 
		 * and std::vector<int>& skeleton_vertices_ must be consistent with the first element 
		 * of the parameter std::vector<int>& corner_vertices_.
		 * .
		 */
		MyParam(const easy3d::SurfaceMesh* orignal_mesh_,
			std::vector<int>& boundary_vertices_,
			std::vector<int>& skeleton_vertices_,
			std::vector<int>& corner_verices_,
			std::vector<double>& distance_param_,
			double& u_d_param_, double& v_d_param);
		~MyParam();

	public:
		//parameterization function
		void Parameterization(int Type);
		//parameterization function - version: nonuniform.
		void Parameterization_nonuniform(int Type);

		//copy param_mesh(result)
		void getParam(easy3d::SurfaceMesh& mesh);

	protected:

		//======================================================================================
		//design for the constructor: MyParam(const easy3d::SurfaceMesh* orignal_mesh_).
		void pre_compute();
		void pre_compute_boundary_vertices();
		void boundary_vertices_squaremapping();
		//======================================================================================

		//======================================================================================
		//design for the constructor: MyParam(const easy3d::SurfaceMesh* orignal_mesh_,
		//std::vector<int>& boundary_vertices_,
		//	std::vector<int>& skeleton_vertices_,
		//	std::vector<int>& corner_verices_);
		void pre_compute_nonuniform();
		void boundary_vertices_rectanglemapping();
		//======================================================================================

		//parameterization methods
		void uniform_weight(int idx, std::vector<double>& lambda);//type 1
		void chord_weight(int idx, std::vector<double>& lambda);//type 2

		//auxiliary functions.
		void convert_float_2_double();
		void convert_tspline_controlmesh_2_easy3d_surfacemesh(const tspline::Tspline& tsp, easy3d::SurfaceMesh& mesh);

		void collect_inner_skeleton_vertices_idx(int& cv1, int& cv2, std::vector<int>& inner_sv);

		//inline functions
		inline int get_n_neighbor_vertex(int idx);
		inline std::vector<int> get_neighbor_idx(int idx);
		inline void convert_v_2_idx(std::vector<int>& inner_sv, std::vector<int>& inner_sv_idx);

	protected:
		const easy3d::SurfaceMesh* orignal_mesh;//default float
		std::vector<easy3d::dvec3> BoundaryPoints;
		std::vector<easy3d::dvec3> InnerPoints;
		std::vector<easy3d::dvec3> AllPoints;

		
		std::vector<int> skeleton_vertices;
		std::vector<int> corner_vertices;
		std::vector<int> boundary_vertices;
		std::vector<int> inner_vertices;
		std::vector<int> all_vertices;

		std::vector<double> distance_param;

		easy3d::SurfaceMesh param_mesh;
		std::vector<easy3d::dvec3> param_coor;//set const z=0.0;
		std::vector<easy3d::dvec3> param_coor_boundary;//set const z=0.0;
		std::vector<easy3d::dvec3> param_coor_inner;//set const z=0.0;

		double u_d_param;
		double v_d_param;


		int n_vertices;
		int n_edges;
		int n_halfedges;
		int n_faces;

	};
}