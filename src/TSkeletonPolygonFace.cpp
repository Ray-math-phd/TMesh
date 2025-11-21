#include "TSkeletonPolygonFace.h"

namespace skeleton {
	//====================================== class BasePolygonModifier ==================================
	TSkeleton::Halfedge_handle BasePolygonModifier::get_first_border_halfedge()const {
		return first_border_halfedge_;
	}

	// ==================================== class SimplePolygonModifier =================================
	void SimplePolygonModifier::judge_mode(int& mode) {
		if (he_ == TSkeleton::Halfedge_handle{}) {
			mode = 0;
		}
		else {
			mode = 1;
		}
	}

	void SimplePolygonModifier::operator()(TSkeleton::HalfedgeDS& hds) {
		int mode = 0;
		judge_mode(mode);
		switch (mode) {
		case 0:
			create_simple_polygon(hds);
			break;
		case 1:
			add_simple_polygon(hds);
			break;
		default:
			create_simple_polygon(hds);
			break;
		}
	}


	void SimplePolygonModifier::create_simple_polygon(TSkeleton::HalfedgeDS& hds) {
		//assume that the size of points_ is greator than 3.
		typedef TSkeleton::HalfedgeDS HDS;
		typedef CGAL::HalfedgeDS_decorator<HDS> Decorator;
		typedef typename HDS::Vertex Vertex;
		typedef typename HDS::Halfedge Halfedge;
		typedef typename Halfedge::Base HBase;

		Decorator decorator(hds);
		int n = points_.size();

		// 1. create the vertices.
		std::vector<typename HDS::Vertex_handle> vertices;
		for (const auto& pt : points_) {
			vertices.push_back(decorator.vertices_push_back(Vertex(pt)));
		}

		// 2. create the loop.
		std::vector<typename HDS::Halfedge_handle> border_loop;
		for (int i = 0; i < n; i++) {
			typename HDS::Halfedge_handle he = hds.edges_push_back(Halfedge(), Halfedge());

			decorator.set_vertex(he, vertices[i]);
			decorator.set_vertex(he->opposite(), vertices[(i + 1) % n]);

			border_loop.push_back(he);
		}

		// 3. connect all halfedges.
		for (int i = 0; i < n; i++) {
			int next_i = (i + 1) % n;
			border_loop[i]->HBase::set_next(border_loop[next_i]);
			decorator.set_prev(border_loop[next_i], border_loop[i]);
		}

		// 4. set vertex's halfedge
		for (int i = 0; i < n; i++) {
			decorator.set_vertex_halfedge(vertices[i], border_loop[i]);
		}

		// 5. save the first border halfedge.(the idx in hds)
		first_border_halfedge_ = border_loop[0];
	}


	void SimplePolygonModifier::add_simple_polygon(TSkeleton::HalfedgeDS& hds) {
		typedef TSkeleton::HalfedgeDS HDS;
		typedef CGAL::HalfedgeDS_decorator<HDS> Decorator;
		typedef typename HDS::Vertex Vertex;
		typedef typename HDS::Halfedge Halfedge;
		typedef typename Halfedge::Base HBase;

		Decorator decorator(hds);
		int n = points_.size();

		auto source_v_ccw = he_->vertex();
		auto target_v_ccw = he_->opposite()->vertex();

		// 1. safe check.
		auto source_v_pos = source_v_ccw->point();
		auto target_v_pos = target_v_ccw->point();
		auto s_v_it = std::find(points_.begin(), points_.end(), source_v_pos);
		auto t_v_it = std::find(points_.begin(), points_.end(), target_v_pos);
		if (s_v_it == points_.end() || t_v_it == points_.end()) {
			std::cout << "" << std::endl;
			return;
		}

		int s_v_idx = std::distance(points_.begin(), s_v_it);
		int t_v_idx = std::distance(points_.begin(), t_v_it);
		int diff = std::abs(s_v_idx - t_v_idx);
		if (diff != 1 && diff != (static_cast<int>(points_.size()) - 1)) {
			std::cout << "" << std::endl;
			return;
		}

		// 2. rotate the points_.
		std::vector<Point3d> resort_points_ = points_;
		std::rotate(resort_points_.begin(), resort_points_.begin() + t_v_idx, resort_points_.end());
		std::vector<bool> resort_points_existed;
		judge_polygon_vertices_ccw_existed(resort_points_, resort_points_existed);


		// 3. add extisted vertices and create new vertices.
		// 3.1 get he_'s HDS::Vertex_handle
		typename HDS::Vertex_handle source_vertex_handle = he_->vertex();
		typename HDS::Vertex_handle target_vertex_handle = he_->opposite()->vertex();

		std::vector<typename HDS::Vertex_handle> vertices;
		vertices.resize(points_.size());
		// add he_'s HDS::Vertex_handle.
		vertices[0] = source_vertex_handle;
		vertices[1] = target_vertex_handle;

		// create new vertices.
		int pt_count(0);
		for (const auto& pt : resort_points_) {
			if (pt_count < 2) {
				pt_count++;
				continue;
			}
			vertices[pt_count]= decorator.vertices_push_back(Vertex(pt));
			pt_count++;
		}

		// 4. create the loop.
		std::vector<typename HDS::Halfedge_handle> border_loop;
		for (int i = 0; i < n; i++) {
			if (i == 0) {
				border_loop.push_back(he_);
			}else if (i != 0 && resort_points_existed[i] == true && resort_points_existed[(i + 1) % n] == true) {
				// reuse existing halfedge if both vertices exist
				Point3d s_pos = resort_points_[i];
				Point3d t_pos = resort_points_[(i + 1) % n];
				TSkeleton::Halfedge_handle existing_he = get_exsited_half_edge_ccw(s_pos, t_pos);
				if (existing_he != TSkeleton::Halfedge_handle{}) {
					border_loop.push_back(existing_he);
				} else {
					// if not found, create new halfedge
					typename HDS::Halfedge_handle he = hds.edges_push_back(Halfedge(), Halfedge());
					decorator.set_vertex(he, vertices[i]);
					decorator.set_vertex(he->opposite(), vertices[(i + 1) % n]);
					border_loop.push_back(he);
				}
			}else {
				// create new halfedges.
				typename HDS::Halfedge_handle he = hds.edges_push_back(Halfedge(), Halfedge());
				decorator.set_vertex(he, vertices[i]);
				decorator.set_vertex(he->opposite(), vertices[(i + 1) % n]);
				border_loop.push_back(he);
			}
		}

		// 5. connect all halfedges.
		for (int i = 0; i < n; i++) {
			int next_i = (i + 1) % n;
			border_loop[i]->HBase::set_next(border_loop[next_i]);
			decorator.set_prev(border_loop[next_i], border_loop[i]);
		}

		// 6. set vertices's halfedge.
		// In CGAL's HalfedgeDS, a vertex's halfedge is just a reference to one of its incident halfedges.
		// Setting it for the new loop won't break the existing loop structure.
		for (int i = 0; i < n; i++) {
			decorator.set_vertex_halfedge(vertices[i], border_loop[i]);
		}

		first_border_halfedge_ = border_loop[0];
	}

	//==================================== axuiliary functions ========================================
	void SimplePolygonModifier::judge_polygon_vertices_ccw_existed(
		std::vector<Point3d>& points__,
		std::vector<bool>& is_existed) {

		// clear for safe.
		is_existed.clear();

		// this funcion will be scoped only and only if the he_ is not the TSkeleton::Halfedge_handle{}.
		// 
		// 1. get the existed point of Skeleton.
		std::vector<Point3d> skeleton_vertices;
		for (auto v_it = skt_.vertices_begin(); v_it != skt_.vertices_end(); v_it++) {
			auto v_pos = v_it->point();
			skeleton_vertices.push_back(v_pos);
		}

		// 2. init the std::vector<bool>& is_existed
		for (int i = 0; i < points__.size(); i++) {
			auto point_it = std::find(skeleton_vertices.begin(), skeleton_vertices.end(), points__[i]);
			if (point_it != skeleton_vertices.end()) {
				is_existed.push_back(true);
			}
			else {
				is_existed.push_back(false);
			}
		}
	}

	TSkeleton::Halfedge_handle SimplePolygonModifier::get_exsited_half_edge_ccw(
		Point3d& s_pos,
		Point3d& t_pos) {
		// For const object, use halfedges() iterator, then get handle through &*he_it
		for (auto he_it = skt_.halfedges_begin(); he_it != skt_.halfedges_end(); ++he_it) {
			// Get non-const handle from const iterator (via const_cast, since we need to return non-const handle)
			TSkeleton::Halfedge_handle he_handle = TSkeleton::Halfedge_handle(const_cast<typename TSkeleton::Halfedge*>(&*he_it));
			auto oppo_he_it = he_handle->opposite();
			auto s_pos_ = oppo_he_it->vertex()->point();
			auto t_pos_ = he_handle->vertex()->point();
			if (s_pos == s_pos_ && t_pos == t_pos_) {
				return he_handle;
			}
			else if (s_pos == t_pos_ && t_pos == s_pos_) {
				return oppo_he_it;
			}
		}
		
		// If no matching edge found, return empty handle
		return TSkeleton::Halfedge_handle{};
	}
}
