#include "MyParam.h"
#include <vector>
#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/Dense>


namespace myparam {

	MyParam::MyParam(const easy3d::SurfaceMesh* orignal_mesh_) :orignal_mesh(orignal_mesh_) {
		n_vertices = orignal_mesh->vertices_size();
		n_edges = orignal_mesh->edges_size();
		n_halfedges = orignal_mesh->halfedges_size();
		n_faces = orignal_mesh->faces_size();

		BoundaryPoints.clear();
		InnerPoints.clear();
		AllPoints.clear();

		boundary_vertices.clear();
		inner_vertices.clear();
		all_vertices.clear();

		param_coor.clear();
		param_coor_boundary.clear();
		param_coor_inner.clear();
	}

	MyParam::MyParam(const easy3d::SurfaceMesh* orignal_mesh_,
		std::vector<int>& boundary_vertices_,
		std::vector<int>& skeleton_vertices_,
		std::vector<int>& corner_verices_,
		std::vector<double>& distance_param_,
		double& u_d_param_, double& v_d_param_) :
		orignal_mesh(orignal_mesh_),
		boundary_vertices(boundary_vertices_),
		skeleton_vertices(skeleton_vertices_),
		corner_vertices(corner_verices_),
		u_d_param(u_d_param_), v_d_param(v_d_param_),
		distance_param(distance_param_){

		n_vertices = orignal_mesh->vertices_size();
		n_edges = orignal_mesh->edges_size();
		n_halfedges = orignal_mesh->halfedges_size();
		n_faces = orignal_mesh->faces_size();

		BoundaryPoints.clear();
		InnerPoints.clear();
		AllPoints.clear();

		inner_vertices.clear();
		all_vertices.clear();

		param_coor.clear();
		param_coor_boundary.clear();
		param_coor_inner.clear();
	}


	MyParam::~MyParam() {
	}

	void MyParam::convert_float_2_double() {

		std::cout << "convert_float_2_double() called - currently empty implementation" << std::endl;
	}

	void MyParam::convert_tspline_controlmesh_2_easy3d_surfacemesh(const tspline::Tspline& tsp, easy3d::SurfaceMesh& mesh) {

		std::cout << "convert_tspline_controlmesh_2_easy3d_surfacemesh() called - currently empty implementation" << std::endl;
	}

	void myparam::MyParam::Parameterization(int Type) {
		int k = 0;
		int matrix_size;
		int n_boundary = boundary_vertices.size();

		std::vector<double> dweight;

		pre_compute();

		matrix_size = inner_vertices.size();
		Eigen::MatrixXd CoeffMatrix = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
		Eigen::VectorXd RightTermx = Eigen::VectorXd::Zero(matrix_size);
		Eigen::VectorXd RightTermy = Eigen::VectorXd::Zero(matrix_size);
		Eigen::VectorXd m_solute_x, m_solute_y;

		std::cout << "solving equation:" << std::endl;

		for (int i = 0; i < inner_vertices.size(); i++) {
			dweight.clear();
			if (Type == 1) {
				uniform_weight(inner_vertices[i], dweight);
			}
			else if (Type == 2) {
				chord_weight(inner_vertices[i], dweight);
			}

			k = get_n_neighbor_vertex(inner_vertices[i]);
			if (k != -1) {
				std::vector<int> neighbor_v_idx = get_neighbor_idx(inner_vertices[i]);

				if (neighbor_v_idx.size() > 0) {
					for (int j = 0; j < k; j++) {
						auto it_b = std::find(boundary_vertices.begin(), boundary_vertices.end(), neighbor_v_idx[j]);
						auto it_i = std::find(inner_vertices.begin(), inner_vertices.end(), neighbor_v_idx[j]);

						if (it_b != boundary_vertices.end()) {
							int idx_in_boundary_vertices = std::distance(boundary_vertices.begin(), it_b);
							if (idx_in_boundary_vertices < param_coor_boundary.size()) {
								double param_x = param_coor_boundary[idx_in_boundary_vertices].x;
								double param_y = param_coor_boundary[idx_in_boundary_vertices].y;
								RightTermx[i] += dweight[j] * param_x;
								RightTermy[i] += dweight[j] * param_y;
							}
						}
						else if (it_i != inner_vertices.end()) {
							int idx_in_inner_vertices = std::distance(inner_vertices.begin(), it_i);
							if (idx_in_inner_vertices < matrix_size) {  
								CoeffMatrix(i, idx_in_inner_vertices) = -dweight[j];
							}
						}
					}
					CoeffMatrix(i, i) = 1.0;
				}
				else {
					std::cout << "error: neighbor_v_idx.size() < 0" << std::endl;
				}
			}
			else {
				std::cout << "error: neighbor_v_idx.size() < 0 ( k < 0 )" << std::endl;
			}
		}

		m_solute_x = CoeffMatrix.lu().solve(RightTermx);
		m_solute_y = CoeffMatrix.lu().solve(RightTermy);

		param_coor_inner.resize(inner_vertices.size());
		for (int i = 0; i < inner_vertices.size(); i++) {
			easy3d::dvec3 param_pos(m_solute_x(i), m_solute_y(i), 0.0);
			param_coor_inner[i] = param_pos;
		}

		param_coor.reserve(param_coor_boundary.size() + param_coor_inner.size());
		param_coor.insert(param_coor.end(), param_coor_boundary.begin(), param_coor_boundary.end());
		param_coor.insert(param_coor.end(), param_coor_inner.begin(), param_coor_inner.end());


		std::map<easy3d::SurfaceMesh::Vertex, easy3d::SurfaceMesh::Vertex> vertex_map;
		for (auto v : orignal_mesh->vertices()) {
			int idx = v.idx();
			auto it_idx = std::find(all_vertices.begin(), all_vertices.end(), idx);
			if (it_idx != all_vertices.end()) {
				int idx_ = std::distance(all_vertices.begin(), it_idx);
				easy3d::vec3 pos = static_cast<easy3d::vec3>(param_coor[idx_]);
				auto new_v = param_mesh.add_vertex(pos);
				vertex_map[v] = new_v;
			}
		}

		for (auto f : orignal_mesh->faces()) {
			if (f.is_valid()) {
				std::vector<easy3d::SurfaceMesh::Vertex> face_vertices;
				for (auto he : orignal_mesh->halfedges(f)) {
					auto v = orignal_mesh->source(he);
					if (vertex_map.find(v) != vertex_map.end()) {
						face_vertices.push_back(vertex_map[v]);
					}
				}
				if (face_vertices.size() >= 3) {
					param_mesh.add_face(face_vertices);
				}
			}
		}
	}

	void MyParam::Parameterization_nonuniform(int Type) {
		int k = 0;
		int matrix_size;
		int n_boundary = boundary_vertices.size();

		std::vector<double> dweight;

		pre_compute_nonuniform();

		matrix_size = inner_vertices.size();
		Eigen::MatrixXd CoeffMatrix = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
		Eigen::VectorXd RightTermx = Eigen::VectorXd::Zero(matrix_size);
		Eigen::VectorXd RightTermy = Eigen::VectorXd::Zero(matrix_size);
		Eigen::VectorXd m_solute_x, m_solute_y;

		std::cout << "solving equation:" << std::endl;

		for (int i = 0; i < inner_vertices.size(); i++) {
			dweight.clear();
			if (Type == 1) {
				uniform_weight(inner_vertices[i], dweight);
			}
			else if (Type == 2) {
				chord_weight(inner_vertices[i], dweight);
			}

			k = get_n_neighbor_vertex(inner_vertices[i]);
			if (k != -1) {
				std::vector<int> neighbor_v_idx = get_neighbor_idx(inner_vertices[i]);

				if (neighbor_v_idx.size() > 0) {
					for (int j = 0; j < k; j++) {
						auto it_b = std::find(boundary_vertices.begin(), boundary_vertices.end(), neighbor_v_idx[j]);
						auto it_i = std::find(inner_vertices.begin(), inner_vertices.end(), neighbor_v_idx[j]);

						if (it_b != boundary_vertices.end()) {
							int idx_in_boundary_vertices = std::distance(boundary_vertices.begin(), it_b);
							if (idx_in_boundary_vertices < param_coor_boundary.size()) {
								double param_x = param_coor_boundary[idx_in_boundary_vertices].x;
								double param_y = param_coor_boundary[idx_in_boundary_vertices].y;
								RightTermx[i] += dweight[j] * param_x;
								RightTermy[i] += dweight[j] * param_y;
							}
						}
						else if (it_i != inner_vertices.end()) {
							int idx_in_inner_vertices = std::distance(inner_vertices.begin(), it_i);
							if (idx_in_inner_vertices < matrix_size) {
								CoeffMatrix(i, idx_in_inner_vertices) = -dweight[j];
							}
						}
					}
					CoeffMatrix(i, i) = 1.0;
				}
				else {
					std::cout << "error: neighbor_v_idx.size() < 0" << std::endl;
				}
			}
			else {
				std::cout << "error: neighbor_v_idx.size() < 0 ( k < 0 )" << std::endl;
			}
		}

		m_solute_x = CoeffMatrix.lu().solve(RightTermx);
		m_solute_y = CoeffMatrix.lu().solve(RightTermy);

		param_coor_inner.resize(inner_vertices.size());
		for (int i = 0; i < inner_vertices.size(); i++) {
			easy3d::dvec3 param_pos(m_solute_x(i), m_solute_y(i), 0.0);
			param_coor_inner[i] = param_pos;
		}

		param_coor.reserve(param_coor_boundary.size() + param_coor_inner.size());
		param_coor.insert(param_coor.end(), param_coor_boundary.begin(), param_coor_boundary.end());
		param_coor.insert(param_coor.end(), param_coor_inner.begin(), param_coor_inner.end());


		std::map<easy3d::SurfaceMesh::Vertex, easy3d::SurfaceMesh::Vertex> vertex_map;
		for (auto v : orignal_mesh->vertices()) {
			int idx = v.idx();
			auto it_idx = std::find(all_vertices.begin(), all_vertices.end(), idx);
			if (it_idx != all_vertices.end()) {
				int idx_ = std::distance(all_vertices.begin(), it_idx);
				easy3d::vec3 pos = static_cast<easy3d::vec3>(param_coor[idx_]);
				auto new_v = param_mesh.add_vertex(pos);
				vertex_map[v] = new_v;
			}
		}

		for (auto f : orignal_mesh->faces()) {
			if (f.is_valid()) {
				std::vector<easy3d::SurfaceMesh::Vertex> face_vertices;
				for (auto he : orignal_mesh->halfedges(f)) {
					auto v = orignal_mesh->source(he);
					if (vertex_map.find(v) != vertex_map.end()) {
						face_vertices.push_back(vertex_map[v]);
					}
				}
				if (face_vertices.size() >= 3) {
					param_mesh.add_face(face_vertices);
				}
			}
		}
	}

	void MyParam::getParam(easy3d::SurfaceMesh& mesh) {
		mesh.clear();
		mesh = param_mesh;
	}

	void MyParam::pre_compute_boundary_vertices() {
		easy3d::SurfaceMesh::Vertex first_v, curr_v;
		bool found_first = false;

		for (auto v : orignal_mesh->vertices()) {
			if (orignal_mesh->is_border(v)) {
				first_v = v;
				curr_v = v;
				found_first = true;
				break;
			}
		}

		if (!found_first) {
			std::cout << "Warning: No boundary vertices found in the mesh." << std::endl;
			return;
		}

		std::vector<int> been_pushed;
		been_pushed.push_back(curr_v.idx());
		boundary_vertices.push_back(curr_v.idx());
		easy3d::dvec3 pos = static_cast<easy3d::dvec3>(orignal_mesh->position(curr_v));
		BoundaryPoints.push_back(pos);

		do {
			bool found_next = false;
			easy3d::SurfaceMesh::Halfedge next_he;

			for (auto he : orignal_mesh->halfedges(curr_v)) {
				if (orignal_mesh->is_border(he)) {
					auto target_v = orignal_mesh->target(he);
					auto it_v = std::find(been_pushed.begin(), been_pushed.end(), target_v.idx());
					if (it_v == been_pushed.end()) {
						been_pushed.push_back(target_v.idx());
						boundary_vertices.push_back(target_v.idx());
						easy3d::dvec3 pos_ = static_cast<easy3d::dvec3>(orignal_mesh->position(target_v));
						BoundaryPoints.push_back(pos_);
						curr_v = target_v;
						found_next = true;
						break;
					}
				}
			}
			if (!found_next) {
				std::cout << "Warning: Boundary is not closed or traversal completed." << std::endl;
				break;
			}
		} while (curr_v != first_v);

		std::cout << "Found " << boundary_vertices.size() << " boundary vertices in order." << std::endl;
	}

	void MyParam::pre_compute() {
		pre_compute_boundary_vertices();
		boundary_vertices_squaremapping();
		for (auto v : orignal_mesh->vertices()) {
			if (orignal_mesh->is_isolated(v)) {
				std::cout << "isolated vertex idx: " << v.idx() << "hasn't been pushed." << std::endl;
				continue;
			}
			if (orignal_mesh->is_border(v)) {
				std::cout << "boundary vertex idx: " << v.idx() << "has been pushed." << std::endl;
				continue;
			}
			else {
				inner_vertices.push_back(v.idx());
				easy3d::vec3 pos = orignal_mesh->position(v);
				easy3d::dvec3 pos_ = static_cast<easy3d::dvec3>(pos);
				InnerPoints.push_back(pos_);
				std::cout << "inner vertex idx: " << v.idx() << "has been pushed." << std::endl;
			}
		}

		all_vertices.reserve(boundary_vertices.size() + inner_vertices.size());
		all_vertices.insert(all_vertices.end(), boundary_vertices.begin(), boundary_vertices.end());
		all_vertices.insert(all_vertices.end(), inner_vertices.begin(), inner_vertices.end());

		AllPoints.reserve(BoundaryPoints.size() + InnerPoints.size());
		AllPoints.insert(AllPoints.end(), BoundaryPoints.begin(), BoundaryPoints.end());
		AllPoints.insert(AllPoints.end(), InnerPoints.begin(), InnerPoints.end());
		std::cout << "all vertices " << "has been pushed." << std::endl;
	}

	void MyParam::boundary_vertices_squaremapping() {
		int n_boundary_vertex = boundary_vertices.size();

		if (n_boundary_vertex < 4) {
			std::cout << "Error: Not enough boundary vertices for square mapping. Need at least 4, got " << n_boundary_vertex << std::endl;
			return;
		}

		param_coor_boundary.clear();
		param_coor_boundary.resize(n_boundary_vertex);

		int remaining_vertices = n_boundary_vertex - 4;
		int vertices_per_side = remaining_vertices / 4;
		int remainder = remaining_vertices % 4;

		int bottom_internal = vertices_per_side + (remainder > 0 ? 1 : 0);
		int right_internal = vertices_per_side + (remainder > 1 ? 1 : 0);
		int top_internal = vertices_per_side + (remainder > 2 ? 1 : 0);
		int left_internal = vertices_per_side;

		int bottom_total = bottom_internal + 1;  // +1 for start point
		int right_total = right_internal + 1;    // +1 for start point
		int top_total = top_internal + 1;        // +1 for start point
		int left_total = left_internal + 1;      // +1 for start point

		if (bottom_total + right_total + top_total + left_total == n_boundary_vertex) {
			std::cout << "function(boundary_vertices_squaremapping): number of boundary is equal." << std::endl;
			std::cout << "function(boundary_vertices_squaremapping): start boundary mapping:" << std::endl;

			int current_idx = 0;

			param_coor_boundary[current_idx] = easy3d::dvec3(0.0, 0.0, 0.0);
			current_idx++;

			for (int i = 1; i <= bottom_internal; i++) {
				double x = static_cast<double>(i) / (bottom_total);
				param_coor_boundary[current_idx] = easy3d::dvec3(x, 0.0, 0.0);
				current_idx++;
			}

			param_coor_boundary[current_idx] = easy3d::dvec3(1.0, 0.0, 0.0);
			current_idx++;

			for (int i = 1; i <= right_internal; i++) {
				double y = static_cast<double>(i) / (right_total);
				param_coor_boundary[current_idx] = easy3d::dvec3(1.0, y, 0.0);
				current_idx++;
			}

			param_coor_boundary[current_idx] = easy3d::dvec3(1.0, 1.0, 0.0);
			current_idx++;

			for (int i = 1; i <= top_internal; i++) {
				double x = 1.0 - static_cast<double>(i) / (top_total);
				param_coor_boundary[current_idx] = easy3d::dvec3(x, 1.0, 0.0);
				current_idx++;
			}

			param_coor_boundary[current_idx] = easy3d::dvec3(0.0, 1.0, 0.0);
			current_idx++;

			for (int i = 1; i <= left_internal; i++) {
				double y = 1.0 - static_cast<double>(i) / (left_total);
				param_coor_boundary[current_idx] = easy3d::dvec3(0.0, y, 0.0);
				current_idx++;
			}

			std::cout << "function(boundary_vertices_squaremapping): finish boundary mapping." << std::endl;
		}
		else {
			std::cout << "function(boundary_vertices_squaremapping): error." << std::endl;
		}
	}

	//======================================================================================================

	void MyParam::pre_compute_nonuniform() {
		boundary_vertices_rectanglemapping();
		BoundaryPoints.resize(boundary_vertices.size());
		//1.load boundary points and inner points.
		for (auto v : orignal_mesh->vertices()) {
			if (orignal_mesh->is_isolated(v)) {
				std::cout << "isolated vertex idx: " << v.idx() << "hasn't been pushed." << std::endl;
				continue;
			}
			if (orignal_mesh->is_border(v)) {
				std::cout << "boundary vertex idx: " << v.idx() << "has been pushed." << std::endl;
				continue;
			}
			else {
				auto boundary_ite = std::find(boundary_vertices.begin(), boundary_vertices.end(), v.idx());
				if (boundary_ite != boundary_vertices.end()) {
					int idx = std::distance(boundary_vertices.begin(), boundary_ite);
					easy3d::vec3 pos = orignal_mesh->position(v);
					easy3d::dvec3 pos_ = static_cast<easy3d::dvec3>(pos);
					BoundaryPoints[idx] = pos_;
//					std::cout << "boundary vertex idx: " << v.idx() << "has been pushed." << std::endl;
				}
				else {// inner points.
					inner_vertices.push_back(v.idx());
					easy3d::vec3 pos = orignal_mesh->position(v);
					easy3d::dvec3 pos_ = static_cast<easy3d::dvec3>(pos);
					InnerPoints.push_back(pos_);
					std::cout << "inner vertex idx: " << v.idx() << "has been pushed." << std::endl;
				}
			}
		}
		all_vertices.reserve(boundary_vertices.size() + inner_vertices.size());
		all_vertices.insert(all_vertices.end(), boundary_vertices.begin(), boundary_vertices.end());
		all_vertices.insert(all_vertices.end(), inner_vertices.begin(), inner_vertices.end());

		AllPoints.reserve(BoundaryPoints.size() + InnerPoints.size());
		AllPoints.insert(AllPoints.end(), BoundaryPoints.begin(), BoundaryPoints.end());
		AllPoints.insert(AllPoints.end(), InnerPoints.begin(), InnerPoints.end());
		std::cout << "all vertices " << "has been pushed." << std::endl;
	}

	void MyParam::boundary_vertices_rectanglemapping() {
		int n_boundary_vertex = boundary_vertices.size();
		
		if (n_boundary_vertex < 4) {
			std::cout << "Error: Not enough boundary vertices for square mapping. Need at least 4, got " << n_boundary_vertex << std::endl;
			return;
		}

		param_coor_boundary.clear();
		param_coor_boundary.resize(n_boundary_vertex);

		//1.date preparations
		std::vector<std::pair<int, int>> edges;
		int n = corner_vertices.size();
		for (int i = 0; i < corner_vertices.size(); i++) {
			edges.push_back(std::pair<int, int>(corner_vertices[i], corner_vertices[(i + 1) % n]));
		}

		//bottom:
		auto cv0_ite = std::find(boundary_vertices.begin(), boundary_vertices.end(), corner_vertices[0]);
		int cv0_idx = std::distance(boundary_vertices.begin(), cv0_ite);
		//bottom inner:
		std::vector<int> bottom_inner_sv;
		collect_inner_skeleton_vertices_idx(corner_vertices[0], corner_vertices[1], bottom_inner_sv);
		std::vector<int> bottom_inner_sv_idx;//will be used
		convert_v_2_idx(bottom_inner_sv, bottom_inner_sv_idx);
		

		//right:
		auto cv1_ite = std::find(boundary_vertices.begin(), boundary_vertices.end(), corner_vertices[1]);
		int cv1_idx = std::distance(boundary_vertices.begin(), cv1_ite);
		//right inner:
		std::vector<int> right_inner_sv;
		collect_inner_skeleton_vertices_idx(corner_vertices[1], corner_vertices[2], right_inner_sv);
		std::vector<int> right_inner_sv_idx;//will be used
		convert_v_2_idx(right_inner_sv, right_inner_sv_idx);

		//top:
		auto cv2_ite = std::find(boundary_vertices.begin(), boundary_vertices.end(), corner_vertices[2]);
		int cv2_idx = std::distance(boundary_vertices.begin(), cv2_ite);
		//top inner:
		std::vector<int> top_inner_sv;
		collect_inner_skeleton_vertices_idx(corner_vertices[2], corner_vertices[3], top_inner_sv);
		std::vector<int> top_inner_sv_idx;//will be used
		convert_v_2_idx(top_inner_sv, top_inner_sv_idx);

		//left:
		auto cv3_ite = std::find(boundary_vertices.begin(), boundary_vertices.end(), corner_vertices[3]);
		int cv3_idx = std::distance(boundary_vertices.begin(), cv3_ite);
		//left inner:
		std::vector<int> left_inner_sv;
		collect_inner_skeleton_vertices_idx(corner_vertices[3], corner_vertices[0], left_inner_sv);
		std::vector<int> left_inner_sv_idx;//will be used
		convert_v_2_idx(left_inner_sv, left_inner_sv_idx);

		//2.Assign coordinate for every boundary vertex
		double bottom_total(0.0), right_total(0.0), top_total(0.0), left_total(0.0);
		int last_sv_idx(-1);
		for (int i = 0, j = 0; i < boundary_vertices.size(); i++) {
			//bottom:
			if (i == cv0_idx) {//orignal point. scv0_idx must be equal to 0.
				param_coor_boundary[i] = easy3d::dvec3(0.0, 0.0, 0.0);
				bottom_total += 0.0;
				last_sv_idx = 0;
			}
			if (cv0_idx < i < cv1_idx) {
				for (int k = 0; k < bottom_inner_sv_idx.size(); k++) {
					if (cv0_idx < i < bottom_inner_sv_idx[0]) {
						//case 1. the first son edge.
						double per_len = distance_param[j] / (bottom_inner_sv_idx[0] - last_sv_idx);
						bottom_total += per_len;
						break;
					}
					else if (bottom_inner_sv_idx[k] < i && (k + 1) < bottom_inner_sv_idx.size() && i < bottom_inner_sv_idx[k + 1]) {
						//case 2. for inner point which not include the skeleton point.
						double per_len = distance_param[j] / (bottom_inner_sv_idx[k] - last_sv_idx);
						bottom_total += per_len;
						break;
					}
					else if (i == bottom_inner_sv_idx[0]) {
						//第一次到达bottom_inner_sv_idx[0]
						double per_len = distance_param[j] / (bottom_inner_sv_idx[k] - last_sv_idx);
						bottom_total += per_len;
						last_sv_idx = bottom_inner_sv_idx[0];
						j++;
						break;
					}
					else if (i == bottom_inner_sv_idx[k]) {
						double per_len = distance_param[j] / (bottom_inner_sv_idx[k] - last_sv_idx);
						bottom_total += per_len;
						last_sv_idx = bottom_inner_sv_idx[k];
						j++;
						break;
					}
					else if (bottom_inner_sv_idx[bottom_inner_sv_idx.size() - 1] < i && i < cv1_idx) {
						double per_len = distance_param[j] / (cv1_idx - last_sv_idx);
						bottom_total += per_len;
						break;
					}
				}
				param_coor_boundary[i] = easy3d::dvec3(bottom_total, 0.0, 0.0);
			}

			//right:
			if (i == cv1_idx) {
				bottom_total += distance_param[j] / (cv1_idx - last_sv_idx);
				param_coor_boundary[i] = easy3d::dvec3(bottom_total, 0.0, 0.0);
				j++;
				last_sv_idx = cv1_idx;
			}
			if (cv1_idx < i < cv2_idx) {
				for (int k = 0; k < right_inner_sv_idx.size(); k++) {
					if (cv0_idx < i < right_inner_sv_idx[0]) {
						//case 1. the first son edge.
						double per_len = distance_param[j] / (right_inner_sv_idx[0] - last_sv_idx);
						right_total += per_len;
						break;
					}
					else if (right_inner_sv_idx[k] < i && (k + 1) < right_inner_sv_idx.size() && i < right_inner_sv_idx[k + 1]) {
						//case 2. for inner point which not include the skeleton point.
						double per_len = distance_param[j] / (right_inner_sv_idx[k] - last_sv_idx);
						right_total += per_len;
						break;
					}
					else if (i == right_inner_sv_idx[0]) {
						//第一次到达right_inner_sv_idx[0]
						double per_len = distance_param[j] / (right_inner_sv_idx[k] - last_sv_idx);
						right_total += per_len;
						last_sv_idx = right_inner_sv_idx[0];
						j++;
						break;
					}
					else if (i == right_inner_sv_idx[k]) {
						double per_len = distance_param[j] / (right_inner_sv_idx[k] - last_sv_idx);
						right_total += per_len;
						last_sv_idx = right_inner_sv_idx[k];
						j++;
						break;
					}
					else if (right_inner_sv_idx[right_inner_sv_idx.size() - 1] < i && i < cv2_idx) {
						double per_len = distance_param[j] / (cv2_idx - last_sv_idx);
						right_total += per_len;
						break;
					}
				}
				param_coor_boundary[i] = easy3d::dvec3(bottom_total, right_total, 0.0);
			}

			//top:
			if (i == cv2_idx) {
				right_total += distance_param[j] - (cv2_idx - last_sv_idx);
				param_coor_boundary[i] = easy3d::dvec3(bottom_total, right_total, 0.0);
				j++;
				last_sv_idx = cv2_idx;
				top_total += bottom_total;//valid because top_total is 0.0.
			}
			if (cv2_idx < i < cv3_idx) {
				for (int k = 0; k < top_inner_sv_idx.size(); k++) {
					if (cv0_idx < i < top_inner_sv_idx[0]) {
						//case 1. the first son edge.
						double per_len = distance_param[j] / (top_inner_sv_idx[0] - last_sv_idx);
						top_total -= per_len;
						break;
					}
					else if (top_inner_sv_idx[k] < i && (k + 1) < top_inner_sv_idx.size() && i < top_inner_sv_idx[k + 1]) {
						//case 2. for inner point which not include the skeleton point.
						double per_len = distance_param[j] / (top_inner_sv_idx[k] - last_sv_idx);
						top_total -= per_len;
						break;
					}
					else if (i == top_inner_sv_idx[0]) {
						//第一次到达right_inner_sv_idx[0]
						double per_len = distance_param[j] / (top_inner_sv_idx[k] - last_sv_idx);
						top_total += per_len;
						last_sv_idx = top_inner_sv_idx[0];
						j++;
						break;
					}
					else if (i == top_inner_sv_idx[k]) {
						double per_len = distance_param[j] / (top_inner_sv_idx[k] - last_sv_idx);
						top_total += per_len;
						last_sv_idx = top_inner_sv_idx[k];
						j++;
						break;
					}
					else if (top_inner_sv_idx[top_inner_sv_idx.size() - 1] < i && i < cv3_idx) {
						double per_len = distance_param[j] / (cv3_idx - last_sv_idx);
						top_total += per_len;
						break;
					}
				}
				param_coor_boundary[i] = easy3d::dvec3(bottom_total, right_total, 0.0);
			}

			//left:
			if (i == cv3_idx) {
				//do the check.
				top_total -= distance_param[j] - (cv3_idx - last_sv_idx);
				if (top_total == 0.0) {
					std::cout << "compute for top is correct." << std::endl;
				}
				param_coor_boundary[i] = easy3d::dvec3(top_total, right_total, 0.0);
				j++;
				last_sv_idx = cv3_idx;
				left_total += right_total;//valid because left_total is 0.0.
			}
			if (cv3_idx < i < boundary_vertices.size()) {
				for (int k = 0; k < left_inner_sv_idx.size(); k++) {
					if (cv0_idx < i < left_inner_sv_idx[0]) {
						//case 1. the first son edge.
						double per_len = distance_param[j] / (left_inner_sv_idx[0] - last_sv_idx);
						left_total -= per_len;
						break;
					}
					else if (left_inner_sv_idx[k] < i && (k + 1) < left_inner_sv_idx.size() && i < left_inner_sv_idx[k + 1]) {
						//case 2. for inner point which not include the skeleton point.
						double per_len = distance_param[j] / (left_inner_sv_idx[k] - last_sv_idx);
						left_total -= per_len;
						break;
					}
					else if (i == left_inner_sv_idx[0]) {
						//第一次到达left_inner_sv_idx[0]
						double per_len = distance_param[j] / (left_inner_sv_idx[k] - last_sv_idx);
						left_total -= per_len;
						last_sv_idx = left_inner_sv_idx[0];
						j++;
						break;
					}
					else if (i == left_inner_sv_idx[k]) {
						double per_len = distance_param[j] / (left_inner_sv_idx[k] - last_sv_idx);
						left_total -= per_len;
						last_sv_idx = left_inner_sv_idx[k];
						j++;
						break;
					}
					else if (left_inner_sv_idx[left_inner_sv_idx.size() - 1] < i && i < boundary_vertices.size() - 1) {
						double per_len = distance_param[j] / (boundary_vertices.size() - 1 - last_sv_idx);
						left_total -= per_len;
						break;
					}
					else if (i == boundary_vertices.size() - 1) {
						double per_len = distance_param[j] / (boundary_vertices.size() - 1 - last_sv_idx);
						left_total -= per_len;
					}
				}
				param_coor_boundary[i] = easy3d::dvec3(bottom_total, right_total, 0.0);
			}
			if (left_total == 0.0) {
				std::cout << "compute for left is correct." << std::endl;
			}
		}
	}

	void MyParam::collect_inner_skeleton_vertices_idx(int& cv1, int& cv2, std::vector<int>& inner_sv) {
		inner_sv.clear();
		auto cv1_ite = std::find(skeleton_vertices.begin(), skeleton_vertices.end(), cv1);
		auto cv2_ite = std::find(skeleton_vertices.begin(), skeleton_vertices.end(), cv2);
		if (cv1_ite != skeleton_vertices.end() && cv2_ite!=skeleton_vertices.end()) {
			int cv1_idx = std::distance(skeleton_vertices.begin(), cv1_ite);
			int cv2_idx = std::distance(skeleton_vertices.begin(), cv2_ite);
			if (cv1_idx < cv2_idx) {
				for (int i = cv1_idx + 1; i < cv2_idx; i++) {
					inner_sv.push_back(skeleton_vertices[i]);
				}
			}
			else {
				for (int i = cv2_idx + 1; i < skeleton_vertices.size(); i++) {
					inner_sv.push_back(skeleton_vertices[i]);
				}
			}
		}
		else {
			std::cout << "" << std::endl;
		}
	}






	//======================================================================================================

	void MyParam::uniform_weight(int idx, std::vector<double>& lambda) {
		auto it_idx = std::find(all_vertices.begin(), all_vertices.end(), idx);
		if (it_idx == all_vertices.end()) {
			std::cout << "input idx error. " << std::endl;
			return;
		}
		auto it_idx_b = std::find(boundary_vertices.begin(), boundary_vertices.end(), idx);
		if (it_idx_b != boundary_vertices.end()) {
			std::cout << "input idx error (boundary vertex). " << std::endl;
			return;
		}

		for (auto v : orignal_mesh->vertices()) {
			if (v.idx() == idx) {
				int k = orignal_mesh->valence(v);
				lambda.resize(k);
				for (int i = 0; i < k; i++) {
					lambda[i] = 1.0 / k;
				}
				std::cout << "uniform weight has beed calculated for vertex idx: " << v.idx() << std::endl;
				return;
			}
		}
	}

	void MyParam::chord_weight(int idx, std::vector<double>& lambda) {
		auto it_idx = std::find(all_vertices.begin(), all_vertices.end(), idx);
		if (it_idx == all_vertices.end()) {
			std::cout << "input idx error. " << std::endl;
			return;
		}
		auto it_idx_b = std::find(boundary_vertices.begin(), boundary_vertices.end(), idx);
		if (it_idx_b != boundary_vertices.end()) {
			std::cout << "input idx error (boundary vertex). " << std::endl;
			return;
		}

		for (auto v : orignal_mesh->vertices()) {
			if (v.idx() == idx) {
				int k = orignal_mesh->valence(v);
				lambda.resize(k);

				std::vector<double> lengths;
				double total_length = 0.0;

				for (auto he : orignal_mesh->halfedges(v)) {
					float length = orignal_mesh->edge_length(he);
					double length_ = static_cast<double>(length);
					lengths.push_back(length_);
					total_length += length_;
				}

				for (int i = 0; i < k; i++) {
					lambda[i] = lengths[i] / total_length;
				}

				std::cout << "chord weight has been calculated for vertex idx: " << v.idx() << std::endl;
				return;
			}
		}
	}

	inline int MyParam::get_n_neighbor_vertex(int idx) {
		int k = -1;
		for (auto v : orignal_mesh->vertices()) {
			if (v.idx() == idx) {
				k = orignal_mesh->valence(v);
				return k;
			}
		}
		return k;
	}

	inline std::vector<int> MyParam::get_neighbor_idx(int idx) {
		std::vector<int> neighbor_idx;
		for (auto v : orignal_mesh->vertices()) {
			if (v.idx() == idx) {
				for (auto he : orignal_mesh->halfedges(v)) {
					auto neighbor_v = orignal_mesh->target(he);
					neighbor_idx.push_back(neighbor_v.idx());
				}
				return neighbor_idx;
			}
		}
		return neighbor_idx;
	}

	inline void MyParam::convert_v_2_idx(std::vector<int>& inner_sv, std::vector<int>& inner_sv_idx) {
		inner_sv_idx.clear();
		for (int i = 0; i < inner_sv.size(); i++) {
			auto v_found = std::find(boundary_vertices.begin(), boundary_vertices.end(), inner_sv[i]);
			if (v_found != boundary_vertices.end()) {
				int idx = std::distance(boundary_vertices.begin(), v_found);
				inner_sv_idx.push_back(idx);
			}
		}
	}
}
