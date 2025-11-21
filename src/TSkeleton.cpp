#include "TSkeleton.h"
#include "TSkeletonPolygonFace.h"

#include <easy3d/core/surface_mesh.h>

#include <queue>
#include <algorithm>
#include <vector>
#include <iostream>

namespace skeleton {
	/**
	 * 输入vid对应的skeleton点必须是一个skeleton face的一个角点！
	 * .
	 */
	void TSkeleton::compute_local_tsp(int& vid) {
		//1. find the target skeleton vertex.
		for (auto vit = vertices_begin(); vit != vertices_end(); vit++) {
			if (vit->id == vid) {//find the target vertex.
				vit->param = Point2d(0.0, 0.0);
				break;
			}
		}
		//2. find the start face.
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			std::vector<int> fit_vertices;
			collect_facet_vertices_idx(fit->id, fit_vertices);
			auto v_found = std::find(fit_vertices.begin(), fit_vertices.end(), vid);
			if (v_found != fit_vertices.end()) {
				// find the v.
				int v_idx = std::distance(fit_vertices.begin(), v_found);

			}
		}
	}


	TSkeleton::Vertex_handle TSkeleton::add_vertex_to_skeleton(const Point3d& pt) {
		typedef TSkeleton::HalfedgeDS HDS;
		typedef CGAL::HalfedgeDS_decorator<HDS> Decorator;
		typedef typename HDS::Vertex Vertex;

		Decorator decorator(this->hds());
		Vertex_handle vh = decorator.vertices_push_back(Vertex(pt));
		vh->is_primary = true;
		return vh;
	}
	TSkeleton::Halfedge_handle TSkeleton::add_triangle_face_to_skeleton(std::vector<TSkeleton::Vertex_handle>& vhs) {
		// assume that input is valid.
		if (vhs.size() != 3) {
			std::cout << "" << std::endl;
			return TSkeleton::Halfedge_handle{};
		}
		auto p0 = vhs[0]->point();
		auto p1 = vhs[1]->point();
		auto p2 = vhs[2]->point();
		if (CGAL::collinear(p0, p1, p2)) {
			std::cout << "警告：三点共线." << std::endl;
		}
		auto vh = this->make_triangle(vhs[0], vhs[1], vhs[2]);
		return vh;
		
	}

	// ================================== compatible check ==========================================
	//可能不用
	bool TSkeleton::check_congruent(std::vector<TSkeleton::Face_handle>& per_facet_vertices_valence) {
		
		for (auto fh = facets_begin(); fh != facets_end(); fh++) {
			TSkeleton::Halfedge_handle he_curr, he_first;
			he_curr = he_first = fh->halfedge();
			std::vector<int> valences;
			do {
				TSkeleton::Vertex_handle v = he_curr->vertex();
				int degree = static_cast<int>(CGAL::circulator_size(v->vertex_begin()) / 2);
				valences.push_back(degree);
				he_curr = he_curr->next();
			} while (he_curr != he_first);
			int size_valence = valences.size();
			int valence_3_count = std::count(valences.begin(), valences.end(), 3);
			int valence_4_count = std::count(valences.begin(), valences.end(), 4);
			if (size_valence != 4) {
				if (valence_3_count != 4 && valence_4_count == 0) {
					return false;
				}
				if (0 < valence_4_count < 4) {
					return false;
				}
			}
		}
		return true;
	}

	// ================================== =========================================
	/**
	 * this function will be scoped after the function get_start_face*().
	 * .
	 */
	
	// default use the id.

	void TSkeleton::init(int& start_fid) {
		std::set<int> been_inited_faces;

		//1.初始化起始面
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == start_fid) {//find the start face.
				//1.1. 获取四个角（方向）边（边的顶点一定是角点）,方向边的两个顶点被保存在c_vertices中，
				//默认为逆时针方向，bottom->right->top->left.
				std::vector<std::pair<int, int>> edges;
				std::vector<int> c_vertices = fit->corner_skeleton_vertices;
				int n = c_vertices.size();
				for (int i = 0; i < c_vertices.size(); i++) {
					edges.push_back(std::pair<int, int>(c_vertices[i], c_vertices[(i + 1) / n]));
				}
				//1.2. 初始化底边的某个子半边对应的参数长度
				TSkeleton::Halfedge_handle curr, first;
				curr = first = fit->halfedge();
				do {
					if (curr->vertex()->id == c_vertices[0]) {//find the vertex.
						curr->d_param = curr->d_geodestic;//default.
						//curr->d_param=2.0;//do for the test.
						break;
					}
					curr = curr->next();
				} while (curr != first);
				//1.3. 初始化起始面的所有半边对应的参数长度
				double d_geodestic(0.0), d_param(0.0);
				double re_d_geodestic(0.0), re_d_param(0.0);
				init_base_he(start_fid, edges[0].first, edges[0].second, d_geodestic, d_param);
				init_adjacent_he_by_he(d_geodestic, d_param, start_fid, edges[1].first, edges[1].second, re_d_geodestic, re_d_param);
				init_opposite_he_by_he(d_geodestic, d_param, start_fid, edges[2].first, edges[2].second);
				init_opposite_he_by_he(re_d_geodestic, re_d_param, start_fid, edges[3].first, edges[3].second);
				//1.4. 
				been_inited_faces.insert(fit->id);
			}
		}
		//1->2.收集与起始面相邻的所有面的id.
		std::queue<int> q;
		q.push(start_fid);

		//2.移除队列q中的首元素，并添加与其相邻的所有面的id
		while (!q.empty()) {
			int s_fid = q.front();
			q.pop();
			std::set<int> adjacent_faces = collect_adjacent_facet_idx(s_fid, been_inited_faces);
			//因为相邻面其某个半边一定被初始化了，因此，使用函数init_base_he(...)就是正确的。
			//2.1. 为adjacent_faces中的每个面进行初始化
			for (auto ite = adjacent_faces.begin(); ite != adjacent_faces.end(); ite++) {
				//为了正确使用init_base_he方法，还需要给出初始的bottom边，即一定有一个半边被初始化的角边的两个顶点。
				//使用init_corner_vertices_idby_fid_hid来得到cv1和cv2，即角边的两个顶点
				//2.1.1 首先寻找对偶半边被赋值的的半边id
				std::vector<int> cvs;
				int hid(-1), fid = *ite;
				for (auto fit = facets_begin(); fit != facets_end(); fit++) {
					if (fit->id == fid) {//find the target face.
						cvs = fit->corner_skeleton_vertices;
						TSkeleton::Halfedge_handle curr, first;
						do {
							if (curr->opposite()->d_param != 0.0) {
								hid = curr->id;
								curr->d_param = curr->opposite()->d_param;
								break;
							}
							curr = curr->next();
						} while (curr != first);
						break;
					}
				}
				//2.1.2 初始化cv1和cv2，并将以cv1，cv2为端点的角边作为底边:bottom
				int cv1(0.0), cv2(0.0);
				init_corner_vertices_id_by_fid_hid(fid, hid, cv1, cv2);
				//2.1.3 构建四个方向边：bottom,right,top,left（默认逆时针）。
				std::vector<std::pair<int, int>> edges;
				edges.push_back(std::pair<int, int>(cv1, cv2));
				std::vector<int>::iterator cv1_ite, cv2_ite, end_ite = cvs.end();
				end_ite--;
				//another implement.
				//for (auto ite = cvs.begin(); ite != cvs.end(); ite++) {
				//	if (*ite == cv1) cv1_ite = ite;
				//	if (*ite == cv2) { cv2_ite = ite; break; }
				//}
				cv1_ite = std::find(cvs.begin(), cvs.end(), cv1);
				cv2_ite = std::find(cvs.begin(), cvs.end(), cv2);
				do {
					if (cv1_ite != cvs.begin()) {
						int second = *cv1_ite;
						cv1_ite--;
						int first = *cv1_ite;
						edges.push_back(std::pair<int, int>(first, second));
					}
					else {
						int second = *cv1_ite;
						cv1_ite = end_ite;
						int first = *cv1_ite;
						edges.push_back(std::pair<int, int>(first, second));
					}
				} while (edges.size() < 4);
				//于是bottom:edges[0],right:edges[1],top:edges[2],left:edges[3].
				//2.1.4 初始化面所有边的参数域长度
				double d_geodestic(0.0), d_param(0.0);
				double re_d_geodestic(0.0), re_d_param(0.0);
				init_base_he(start_fid, edges[0].first, edges[0].second, d_geodestic, d_param);
				init_adjacent_he_by_he(d_geodestic, d_param, start_fid, edges[1].first, edges[1].second, re_d_geodestic, re_d_param);
				init_opposite_he_by_he(d_geodestic, d_param, start_fid, edges[2].first, edges[2].second);
				init_opposite_he_by_he(re_d_geodestic, re_d_param, start_fid, edges[3].first, edges[3].second);
				been_inited_faces.insert(fid);
				q.push(fid);
			}
		}

	}

	void TSkeleton::init_opposite_he_by_he(double& d_geodestic, double& d_param, int& fid, int& cv1, int& cv2) {
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {// find the target face.
				//1. 按逆时针搜寻指定角半边的所有子半边id
				std::vector<int> vertices;
				collect_he_skeleton_vertices_idx(fid, cv1, cv2, vertices);
				//2.将点列转换为半边列
				std::vector<int> heid_ccw;
				collect_facet_halfedges_id(fid, vertices, heid_ccw);
				//3.为目标边赋予全局一致的参数域长度
				int i(0);
				double sum_d_param(0.0);
				TSkeleton::Halfedge_handle curr_he, first_he;
				curr_he = first_he = fit->halfedge();
				do {
					if (curr_he->id == heid_ccw[i] && i < heid_ccw.size() - 1) {//起始边开始,但是不是终止边
						double d_param_ = curr_he->d_geodestic * d_param / d_geodestic;
						curr_he->d_param = d_param_;
						sum_d_param += d_param_;
						i++;
					}
					if (i == heid_ccw.size() - 1) {//终止半边的索引.
						curr_he->d_param = d_param - sum_d_param;
						i++;
					}
					curr_he = curr_he->next();
				} while (i < heid_ccw.size());
			}
		}
	}

	void TSkeleton::init_adjacent_he_by_he(double& d_geodestic, double& d_param, int& fid, int& cv1, int& cv2, double& re_d_geodestic, double& re_d_param) {
		re_d_geodestic = 0.0, re_d_param = 0.0;
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {// find the target face.
				//1. 按逆时针搜寻指定角半边的所有子半边id
				std::vector<int> vertices;
				collect_he_skeleton_vertices_idx(fid, cv1, cv2, vertices);
				//2.将点列转换为半边列
				std::vector<int> heid_ccw;
				collect_facet_halfedges_id(fid, vertices, heid_ccw);
				//3.为目标边赋予全局一致的参数域长度
				int i(0);
				double sum_d_param(0.0);
				TSkeleton::Halfedge_handle curr_he, first_he;
				curr_he = first_he = fit->halfedge();
				do {
					if (curr_he->id == heid_ccw[i]) {//起始边开始,一直到终止边
						double d_param_ = curr_he->d_geodestic * d_param / d_geodestic;
						curr_he->d_param = d_param_;
						sum_d_param += d_param_;
						re_d_geodestic += curr_he->d_geodestic;
						re_d_param += curr_he->d_param;
						i++;
					}
					curr_he = curr_he->next();
				} while (i < heid_ccw.size());
			}
		}
	}
	void TSkeleton::init_adjacent_he_by_he(double& d_geodestic, double& d_param, int& fid, int& cv1, int& cv2) {
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {// find the target face.
				//1. 按逆时针搜寻指定角半边的所有子半边id
				std::vector<int> vertices;
				collect_he_skeleton_vertices_idx(fid, cv1, cv2, vertices);
				//2.将点列转换为半边列
				std::vector<int> heid_ccw;
				collect_facet_halfedges_id(fid, vertices, heid_ccw);
				//3.为目标边赋予全局一致的参数域长度
				int i(0);
				double sum_d_param(0.0);
				TSkeleton::Halfedge_handle curr_he, first_he;
				curr_he = first_he = fit->halfedge();
				do {
					if (curr_he->id == heid_ccw[i]) {//起始边开始,一直到终止边
						double d_param_ = curr_he->d_geodestic * d_param / d_geodestic;
						curr_he->d_param = d_param_;
						sum_d_param += d_param_;
						i++;
					}
					curr_he = curr_he->next();
				} while (i < heid_ccw.size());
			}
		}
	}

	/**
	 * input:int& fid, int& cv1, int& cv2
	 * output:double& d_geodestic, double& d_param
	 * .
	 */
	void TSkeleton::init_base_he(int& fid, int& cv1, int& cv2, double& d_geodestic, double& d_param) {
		d_geodestic = 0.0, d_param = 0.0;
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {//find the target face.
				//1. 按逆时针搜寻指定角半边的所有子半边id
				std::vector<int> vertices;
				collect_he_skeleton_vertices_idx(fid, cv1, cv2, vertices);
				//2.将点列转换为半边列
				std::vector<int> heid_ccw;
				collect_facet_halfedges_id(fid, vertices, heid_ccw);
				//3.找到一个非零边，假定由输入cv1,cv2确定的角边一定存在一个d_param非零的半边.
				double base_d_geodestic(0.0), base_d_param(0.0);
				TSkeleton::Halfedge_handle curr, first;
				curr = first = fit->halfedge();
				do {
					int curr_he_id = curr->id;
					auto ite = std::find(heid_ccw.begin(), heid_ccw.end(), curr_he_id);
					if (ite != heid_ccw.end()) {
						if (curr->opposite()->d_param != 0.0) {
							curr->d_param = curr->opposite()->d_param;//其实这里不为其赋值也没事，第四步会自动处理。
							base_d_geodestic = curr->d_geodestic;
							base_d_geodestic = curr->opposite()->d_param;
							break;
						}
					}
					curr = curr->next();
				} while (curr != first);
				//4.为目标边赋予全局一致的参数域长度
				int i(0);
				double sum_d_param(0.0);
				TSkeleton::Halfedge_handle curr_he, first_he;
				curr_he = first_he = fit->halfedge();
				do {
					//need to modify.
					if (curr_he->id == heid_ccw[i]) {//起始边开始,一直到终止边
						double d_param_ = curr_he->d_geodestic * base_d_param / base_d_geodestic;
						curr_he->d_param = d_param_;
						sum_d_param += d_param_;
						d_geodestic += curr_he->d_geodestic;
						d_param += d_param_;
						i++;
					}
					curr_he = curr_he->next();
				} while (i < heid_ccw.size());
			}
		}
	}

	void TSkeleton::collect_he_skeleton_vertices_idx(int& fid, int& cv1, int& cv2, std::vector<int>& vertices) {
		vertices.clear();
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {
				std::vector<int> face_vertices;//loop.
				collect_facet_vertices_idx(fit->id, face_vertices);
				std::vector<int>::iterator curr;
				bool found = false;
				bool ended = false;
				curr = face_vertices.begin();
				do {
					if (*curr == cv1) {//起始点
						found = true;
						vertices.push_back(*curr);
					}
					if (found == true && *curr != cv1 && *curr != cv2) {//中间点
						vertices.push_back(*curr);
					}
					if (found == true && *curr == cv2) {//终止点
						vertices.push_back(*curr);
						ended = true;
						break;
					}
				} while (++curr != face_vertices.end());
				if (!ended) {//终止点没有被正确加入，重新循环，加入包括终止点以前的点
					curr = face_vertices.begin();
					do {
						if (*curr != cv2) {
							vertices.push_back(*curr);
						}
						if (*curr == cv2) {
							vertices.push_back(*curr);
							ended = true;
							break;
						}
					} while (++curr != face_vertices.end());
				}
			}
		}
	}

	void TSkeleton::init_corner_vertices_id_by_fid_hid(int& fid, int& hid, int& cv1, int& cv2){
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {
				//1. data preparatioins
				std::vector<int> cvs = fit->corner_skeleton_vertices;
				std::vector<int> fvs; collect_facet_vertices_idx(fid, fvs);
				//2.获取hid的两个端点的id，其一定在fvs中
				int svid(-1), tvid(-1);
				TSkeleton::Halfedge_handle curr, first;
				curr = first = fit->halfedge();
				do {
					if (hid == curr->id) {
						svid = curr->vertex()->id;
						tvid = curr->opposite()->vertex()->id;
						break;
					}
					curr = curr->next();
				} while (curr != first);
				//3.寻找cv1,cv2
				//3.1 首先寻找到svid在fvs中的迭代器和tvid在fvs中的迭代器
				std::vector<int>::iterator site, tite, end_ite = fvs.end();
				end_ite--;
				for (auto ite = fvs.begin(); ite != fvs.end(); ite++) {
					if (*ite == svid)site = ite;
					if (*ite == tvid) {
						tite = ite;
						break;
					}
				}
				//3.2 从site开始向上寻找cv1
				bool cv1_found(false);
				while (!cv1_found) {
					auto cv1_it = std::find(cvs.begin(), cvs.end(), *site);
					if (cv1_it != cvs.end()) {//find the cv1.
						cv1 = *site;
						cv1_found = true;
						break;
					}
					else {
						if (site == fvs.begin()) {
							site = end_ite;//从底层开始寻找，因为实际是个环
						}
						else {
							site--;//向上寻找
						}
					}
				}
				//3.3从tite开始向下寻找cv2.
				bool cv2_found(false);
				while (!cv2_found) {
					auto cv2_it = std::find(cvs.begin(), cvs.end(), *tite);
					if (cv2_it != cvs.end()) {//find the cv2.
						cv2 = *tite;
						cv2_found;
						break;
					}
					else {
						if (tite == end_ite) {
							tite = fvs.begin();//从上层开始寻找，因为是个环
						}
						else {
							tite--;
						}
					}
				}
			}
		}
	}

	/*
		may need to return int?
	*/
	TSkeleton::Face_handle TSkeleton::get_start_face() {
		TSkeleton::Face_handle final_fh = TSkeleton::Face_handle{};
		int final_fh_size(0);

		int f_id = 0;
		for (auto fh = facets_begin(); fh != facets_end(); fh++) {
			std::vector<int> valences = compute_facet_valence(f_id);
			int n_valence_4 = std::count(valences.begin(), valences.end(), 4);
			if (valences.size() == 4) {// first select the regular face.
				if (n_valence_4 == 4) {
					return fh;
				}
				else {
					// whenever case, first select the regular face.
					final_fh = fh;// temp.
					final_fh_size = valences.size();
				}
			}
			else if(valences.size() > 4){// the case the size of valences is not equal to 4, which means that the size of ... is greator than 4.
				//second select the not regular face.
				if (final_fh != TSkeleton::Face_handle{} && final_fh_size == 4) {
					//skip.
				}
				else if (final_fh == TSkeleton::Face_handle{}) {
					final_fh = fh;// temp.
					final_fh_size = valences.size();
				}
				else if (final_fh != TSkeleton::Face_handle{} && final_fh_size > fh->size()) {
					final_fh = fh;
					final_fh_size = valences.size();
				}
			}
			else {
				std::cout << "" << std::endl;
			}
			f_id++;
		}
		return final_fh;
	}
	// default use this version.
	int TSkeleton::get_start_face_idx() {
		TSkeleton::Face_handle final_fh = TSkeleton::Face_handle{};
		int final_fh_id(-1);
		int final_fh_size(0);

		int f_id = 0;
		for (auto fh = facets_begin(); fh != facets_end(); fh++) {
			std::vector<int> valences = compute_facet_valence(f_id);
			int n_valence_4 = std::count(valences.begin(), valences.end(), 4);
			if (valences.size() == 4) {// first select the regular face.
				if (n_valence_4 == 4) {
					return f_id;
				}
				else {
					// whenever case, first select the regular face.
					final_fh = fh;// temp.
					final_fh_id = f_id;
					final_fh_size = valences.size();
				}
			}
			else if (valences.size() > 4) {// the case the size of valences is not equal to 4, which means that the size of ... is greator than 4.
				//second select the not regular face.
				if (final_fh != TSkeleton::Face_handle{} && final_fh_size == 4) {
					//skip.
				}
				else if (final_fh == TSkeleton::Face_handle{}) {
					final_fh = fh;// temp.
					final_fh_id = f_id;
					final_fh_size = valences.size();
				}
				else if (final_fh != TSkeleton::Face_handle{} && final_fh_size > fh->size()) {
					final_fh = fh;
					final_fh_id = f_id;
					final_fh_size = valences.size();
				}
			}
			else {
				std::cout << "" << std::endl;
			}
			f_id++;
		}
		return final_fh_id;
	}

	std::vector<double> TSkeleton::compute_skeleton_vertices_d_geodestic(
		easy3d::SurfaceMesh& mesh,
		TSkeleton::Face_handle& fh) {
		std::vector<double> ds_g;
		TSkeleton::Halfedge_handle first_he, curr_he;
		curr_he = first_he = fh->halfedge();
		do {
			std::map<int, int> part_boundary_vertices = curr_he->boundary_vertices;
			std::vector<int> part_rootmesh_vertices; part_rootmesh_vertices.clear();
			for (auto map_it : part_boundary_vertices) {
				part_rootmesh_vertices.push_back(map_it.second);
			}
			double d_g = compute_gedestic_distance(mesh, part_rootmesh_vertices);
			ds_g.push_back(d_g);
		} while (++curr_he != first_he);
	}

	double TSkeleton::compute_halfedge_d_geodestic(easy3d::SurfaceMesh& mesh, TSkeleton::Halfedge_handle& he) {
		// assume that all input are valid.
		for (auto he_it = halfedges_begin(); he_it != halfedges_end(); he_it++) {
			TSkeleton::Vertex_handle s_vh = he_it->vertex();
			TSkeleton::Vertex_handle t_vh = he_it->opposite()->vertex();
			if (s_vh->id == he->vertex()->id && t_vh->id == he->opposite()->vertex()->id) {
				//find the target he.
				auto boundary_vertices_map = he_it->boundary_vertices;
				std::vector<int> vertices;
				vertices.reserve(boundary_vertices_map.size());
				std::transform(
					boundary_vertices_map.begin(), boundary_vertices_map.end(),
					std::back_inserter(vertices),
					[](const std::pair<int, int>& p) {return p.first; });
				double d_g = compute_gedestic_distance(mesh, vertices);
				return d_g;
			}
		}
		return 0.0;
	}

	double TSkeleton::compute_gedestic_distance(easy3d::SurfaceMesh& mesh, std::vector<int>& vertices) {
		// assume that input are valid.
		double gd(0.0);
		for (int i = 0; i < vertices.size() - 1; i++) {
			if (i + 1 <= vertices.size() - 1) {//valid idx.
				double per_gd = get_halfedge_length(mesh, vertices[i], vertices[i + 1]);
				gd += per_gd;
			}
		}
		return gd;
	}

	std::vector<int> TSkeleton::compute_facet_valence(int& fh_count) {
		int count(0);
		std::vector<int> valences; valences.clear();
		for (auto fh_ = facets_begin(); fh_ != facets_end(); fh_++) {
			if (count == fh_count) {
				TSkeleton::Halfedge_handle first_he, curr_he;
				curr_he = first_he = fh_->halfedge();
				do {
					TSkeleton::Vertex_handle v = curr_he->vertex();
					int degree = static_cast<int>(CGAL::circulator_size(v->vertex_begin()) / 2);
					valences.push_back(degree);
				} while (++curr_he != first_he);
			}
			count++;
		}
		return valences;
	}

	std::set<int> TSkeleton::collect_adjacent_facet_idx(int& fid, std::set<int>& been_init_facet) {
		// assume that the input: been_init_facet is not empty.
		std::set<int> adjacent_facets;
		adjacent_facets.clear();

		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {
				TSkeleton::Halfedge_handle curr, first;
				curr = first = fit->halfedge();
				do {
					int ad_fid = curr->opposite()->f_id;
					auto ite = been_init_facet.find(ad_fid);
					if (ite == been_init_facet.end()) {//未被添加过
						adjacent_facets.insert(ad_fid);
					}
					curr = curr->next();
				} while (curr != first);
			}
		}

		return adjacent_facets;
	}
	/**
	 * 函数被调用当且仅当给dual_he的对偶半边所对应的面中的每一个半边的d_param都没有被赋值。
	 * difficult.
	 * .
	 */
	double TSkeleton::compute_consistency_d_param(TSkeleton::Halfedge_handle& dual_he) {
		
	
	}


	// ================================================ inline functions =====================================================

	inline TSkeleton::Vertex_handle TSkeleton::get_vh_by_id(int& id)const {
		typedef TSkeleton::HalfedgeDS HDS;
		
		HDS& hds = const_cast<HDS&>(this->hds());
		
		for (auto he_it = hds.halfedges_begin(); he_it != hds.halfedges_end(); he_it++) {
			auto vh = he_it->vertex();
			if (vh->id == id) {
				return vh;
			}
		}
		
		return TSkeleton::Vertex_handle{};
	}

	inline void TSkeleton::collect_facet_vertices_idx(int& f_id, std::vector<int>& facet_vertices) {
		facet_vertices.clear();
		int f_count(0);
		for (auto fh_it = facets_begin(); fh_it != facets_end(); fh_it++) {
			if (f_count == f_id) {
				TSkeleton::Halfedge_handle curr_he, first_he;
				curr_he = first_he = fh_it->halfedge();
				do {
					TSkeleton::Vertex_handle v = curr_he->vertex();
					facet_vertices.push_back(v->id);
				} while (++curr_he != first_he);
			}
			f_count++;
		}
	}

	inline void TSkeleton::collect_facet_corner_vertices_idx(int& f_id, std::vector<int>& facet_corner_vertices) {
		facet_corner_vertices.clear();
		//1. extract.
		std::vector<int> facet_corner_vertices_;
		std::vector<int> valences;
		int f_count(0);
		for (auto fh_it = facets_begin(); fh_it != facets_end(); fh_it++) {
			if (f_count == f_id) {
				
				TSkeleton::Halfedge_handle curr_he, first_he;
				curr_he = first_he = fh_it->halfedge();
				do {
					TSkeleton::Vertex_handle v = curr_he->vertex();
					int v_valence= static_cast<int>(CGAL::circulator_size(v->vertex_begin()) / 2);
					valences.push_back(v_valence);
					facet_corner_vertices_.push_back(v->id);

				} while (++curr_he != first_he);
			}
			f_count++;
		}

		//2. init.
		if (facet_corner_vertices_.size() == 4) {
			facet_corner_vertices = facet_corner_vertices_;
		}
		else if (facet_corner_vertices_.size() > 4) {
			int n_v_4 = std::count(valences.begin(), valences.end(), 4);
			if (n_v_4 == 4) {
				for (int i = 0; i < facet_corner_vertices_.size(); i++) {
					if (valences[i] == 4) {
						facet_corner_vertices.push_back(facet_corner_vertices_[i]);
					}
				}
			}
			else {
				std::cout << "" << std::endl;
			}
			
		}
		else {
			std::cout << "" << std::endl;
		}
	}

	inline void TSkeleton::collect_facet_halfedges_id(int& fid, std::vector<int>& vertices, std::vector<int>& he_ids) {
		he_ids.clear();
		for (auto fit = facets_begin(); fit != facets_end(); fit++) {
			if (fit->id == fid) {
				int i(0);
				TSkeleton::Halfedge_handle curr, first;
				curr = first = fit->halfedge();
				do {
					int svid = curr->vertex()->id, tvid = curr->opposite()->vertex()->id;
					if (svid == vertices[i] && i + 1 < vertices.size() && tvid == vertices[i + 1]) {//起始边
						he_ids.push_back(curr->id);
						i++;
					}
					curr = curr->next();//充分循环.
				} while (he_ids.size() < vertices.size() - 1);
			}
		}

	}

	inline double get_halfedge_length(easy3d::SurfaceMesh& mesh, int& v1_id, int& v2_id) {
		for (auto v_ite : mesh.vertices()) {
			if (v_ite.idx() == v1_id) {
				easy3d::SurfaceMesh::Halfedge first_he, curr_he;
				first_he = mesh.out_halfedge(v_ite);
				curr_he = first_he;
				
				do {
					if (mesh.target(curr_he).idx() == v2_id) {
						auto p1 = mesh.position(mesh.source(curr_he));
						auto p2 = mesh.position(mesh.target(curr_he));
						return (p2 - p1).norm();
					}
					curr_he = mesh.next_around_source(curr_he); 
				} while (curr_he != first_he);
			}
		}
		return 0.0;
	}

	//================================================= Creator Functions ====================================================

















	//================================================= SKTMesh construct Functions ====================================================
	void TSkeletonCreator::construct_new(TSkeleton& tsk) {
		//0. data preparations 
		//0.1 the deepcopy of class var: rootmesh, meshes.
		easy3d::SurfaceMesh rootmesh_; deepcopy(rootmesh_);
		std::vector<easy3d::SurfaceMesh> meshes_; deepcopy(meshes_);
		//0.2 the map: per patch's(meshes[i], i is valid) boundary vertices -> the rootmesh(rootmesh_).
		std::vector<std::map<int, int>> all_patch_vertices_map;//the head is not the SKTVertex.
		extract_all_patch_vertices_map(all_patch_vertices_map);
		//0.3 the skeleton vertices(idx).
		std::vector<int> skeleton_vertices_idx;
		std::vector<std::map<int, int>> all_patch_skeleton_vertices_idx;
		collect_skeleton_vertices_idx(all_patch_vertices_map, skeleton_vertices_idx, all_patch_skeleton_vertices_idx);
		//0.4 resort the all_patch_vertices_map, makesure that the head is the SKTVertex.
		resort_boundary_vertices_counterclockwise(all_patch_vertices_map, all_patch_skeleton_vertices_idx);//resort the boundary vertices.



		std::map<int, TSkeleton::Vertex_handle> vh_map;

		//1. add all vertices into tsk.
		for (int i = 0; i < skeleton_vertices_idx.size();i++) {
			auto v = get_mesh_vertex_by_idx(rootmesh_, skeleton_vertices_idx[i]);
			auto v_pos = static_cast<easy3d::dvec3>(rootmesh_.position(v));
			Point3d pt = Point3d(v_pos.x, v_pos.y, v_pos.z);
			auto vh = tsk.add_vertex_to_skeleton(pt);
			vh->id = i;
			vh_map.emplace(skeleton_vertices_idx[i], vh);
		}

		//2. construct all patch which form the total skeleton mesh.
		for (int i = 0; i < all_patch_skeleton_vertices_idx.size(); i++) {
			//per patch.
			int merge_count(0);
			TSkeleton::Halfedge_handle merge_vh;
			TSkeleton::Vertex_handle p0_vh;
			auto per_patch_skeleton_vertices_idx = all_patch_skeleton_vertices_idx[i];
			auto p0_root_idx = per_patch_skeleton_vertices_idx[0];
			auto p0_vh_it = std::find(vh_map.begin(), vh_map.end(), p0_root_idx);
			if (p0_vh_it != vh_map.end()) {
				p0_vh = p0_vh_it->second;
				for (int j = 1; j < per_patch_skeleton_vertices_idx.size(); j++) {
					int k = j + 1;
					if (k <= per_patch_skeleton_vertices_idx.size()) {
						//merge internel edge.
						if (merge_count >= 1 && merge_count <= per_patch_skeleton_vertices_idx.size() - 2) {
							tsk.join_facet(merge_vh);
						}
						auto p1_root_idx = per_patch_skeleton_vertices_idx[j];
						auto p2_root_idx = per_patch_skeleton_vertices_idx[k];
						auto p1_vh_it = std::find(vh_map.begin(), vh_map.end(), p1_root_idx);
						auto p2_vh_it = std::find(vh_map.begin(), vh_map.end(), p2_root_idx);
						if (p1_vh_it != vh_map.end() && p2_vh_it != vh_map.end()) {
							auto p1_vh = p1_vh_it->second;
							auto p2_vh = p2_vh_it->second;
							std::vector<TSkeleton::Vertex_handle> vhs = { p0_vh,p1_vh,p2_vh };
							merge_vh = tsk.add_triangle_face_to_skeleton(vhs)->prev();
							if (j >= 2 && j <= per_patch_skeleton_vertices_idx.size()) {
								merge_count++;
							}
						}
					}
				}
			}
			else {
				std::cout << "" << std::endl;
				break;
			}
			
		}

		//init per skeleton face's id.
		int f_id = 0;
		for (auto fh_it = tsk.facets_begin(); fh_it != tsk.facets_end(); fh_it++) {
			fh_it->id = f_id;
			f_id++;
		}
		f_id = 0;

		for (auto fh_it = tsk.facets_begin(); fh_it != tsk.facets_end(); fh_it++) {
			//do for per facet.
			TSkeleton::Halfedge_handle curr_he, first_he;
			curr_he = first_he = fh_it->halfedge();
			do {
				curr_he->f_id = f_id;
				curr_he = curr_he->next();
			} while (curr_he != first_he);

			f_id++;
		}

		f_id = 0;
		//3. init the patch mesh, patch vertices map and the corner vertices for per face.
		//rewrite.
		//int f_id = 0;
		for (auto fh_it = tsk.facets_begin(); fh_it != tsk.facets_end(); fh_it++) {
			fh_it->mesh = meshes[f_id];// the pacth mesh.
			fh_it->vertices_idx_map = all_patch_vertices_map[f_id];// the map: vertices on patch -> vertices on rootmesh.
			// init the corner_vertices and corner_skeleton_vertices.
			// the corner_vertices are the vertices on the patch.
			// the corner_skeleton_vertices are the vertices on the TSkeleton object: tsk.

			//1. init the fh_it's corner_skeleton_vertices.
			std::vector<int> corner_skeleton_vertices;
			tsk.collect_facet_corner_vertices_idx(f_id, corner_skeleton_vertices);
			fh_it->corner_skeleton_vertices = corner_skeleton_vertices;

			//2. init the fh_it's corner_vertices which idx on the patch.
			std::vector<int> corner_vertices;
			std::vector<easy3d::dvec3> corner_vertices_pos;
			for (int i = 0; i < corner_skeleton_vertices.size(); i++) {
				for (auto vh_it = tsk.vertices_begin(); vh_it != tsk.vertices_end(); vh_it++) {
					if (vh_it->id == corner_skeleton_vertices[i]) {
						auto vh_pos = vh_it->point();
						auto vh_pos_dev3 = easy3d::dvec3(vh_pos.x(), vh_pos.y(), vh_pos.z());
						auto corner_vertex_id = get_mesh_vertex_idx_by_pos(rootmesh_, vh_pos_dev3);
						corner_vertices.push_back(corner_vertex_id);
					}
				}
			}
			fh_it->corner_vertices = corner_vertices;

			//3. update the f_id.
			f_id++;
		}

		//4. init per edge's boundary_vertices.
		f_id = 0;
		for (auto fh_it = tsk.facets_begin(); fh_it != tsk.facets_end(); fh_it++) {
			//per patch
			std::map<int, int> boundary_vertices_map = fh_it->vertices_idx_map;
			
			TSkeleton::Halfedge_handle curr_he, first_he;
			curr_he = first_he = fh_it->halfedge();
			do {
				//per patch's per halfedge.
				TSkeleton::Vertex_handle s_v = curr_he->vertex();//source vertex
				TSkeleton::Vertex_handle t_v = curr_he->opposite()->vertex();//target vertex
				Point3d s_v_pos = s_v->point();
				Point3d t_v_pos = t_v->point();
				auto s_v_pos_dvec3 = easy3d::dvec3(s_v_pos.x(), s_v_pos.y(), s_v_pos.z());
				auto t_v_pos_dvec3 = easy3d::dvec3(t_v_pos.x(), t_v_pos.y(), t_v_pos.z());
				int s_v_idx = get_mesh_vertex_idx_by_pos(meshes_[f_id], s_v_pos_dvec3);// the idx on the patch.
				int t_v_idx = get_mesh_vertex_idx_by_pos(meshes_[f_id], t_v_pos_dvec3);// the idx on the patch.

				std::map<int, int> boundary_vertices;
				boundary_vertices.clear();
				
				//1. find the s(t)_v_idx's ite.
				std::map<int, int>::iterator s_v_ite, t_v_ite;
				auto s_v_ite = boundary_vertices_map.find(s_v_idx);
				auto t_v_ite = boundary_vertices_map.find(t_v_idx);

				//2. init the boundary_vertices.
				bool is_finished(false);
				do {
					boundary_vertices.emplace(s_v_ite->first, s_v_ite->second);
					s_v_ite++;
				} while (s_v_ite != t_v_ite && s_v_ite == boundary_vertices_map.end());
				if (s_v_ite == boundary_vertices_map.end()) {
					auto v_ite = boundary_vertices_map.begin();
					if (v_ite == t_v_ite) {
						boundary_vertices.emplace(v_ite->first, v_ite->second);
						//finish
					}
					else {
						do {
							boundary_vertices.emplace(v_ite->first, v_ite->second);
							v_ite++;
						} while (v_ite != t_v_ite);
						boundary_vertices.emplace(v_ite->first, v_ite->second);
						//finish
					}
				}
				else if (s_v_ite == t_v_ite) {
					boundary_vertices.emplace(t_v_ite->first, t_v_ite->second);
				}
				//curr_he->boundary_vertices = boundary_vertices;
			} while (++curr_he != first_he);

			f_id++;
		}

		//5. init the geodestic length for per halfedge.
		for (auto fh_it = tsk.facets_begin(); fh_it != tsk.facets_end(); fh_it++) {
			TSkeleton::Halfedge_handle curr_he, first_he;
			curr_he = first_he = fh_it->halfedge();
			do {
				double d = tsk.compute_halfedge_d_geodestic(*fh_it->mesh.get(), curr_he);
				curr_he->d_param = d;
				curr_he->d_geodestic = d;
				curr_he = curr_he->next();
			} while (curr_he != first_he);
		}

	}

	void TSkeletonCreator::init() {
		
	}

	// not use.
	void TSkeletonCreator::construct(TSkeleton& tsk) {
		//0. data preparations 
		//0.1 the deepcopy of class var: rootmesh, meshes.
		easy3d::SurfaceMesh rootmesh_; deepcopy(rootmesh_);
		std::vector<easy3d::SurfaceMesh> meshes_; deepcopy(meshes_);
		//0.2 the map: per patch's(meshes[i], i is valid) boundary vertices -> the rootmesh(rootmesh_).
		std::vector<std::map<int, int>> all_patch_vertices_map;//the head is not the SKTVertex.
		extract_all_patch_vertices_map(all_patch_vertices_map);
		//0.3 the skeleton vertices(idx).
		std::vector<int> skeleton_vertices_idx;
		std::vector<std::map<int, int>> all_patch_skeleton_vertices_idx;
		collect_skeleton_vertices_idx(all_patch_vertices_map, skeleton_vertices_idx, all_patch_skeleton_vertices_idx);
		//0.4 resort the all_patch_vertices_map, makesure that the head is the SKTVertex.
		resort_boundary_vertices_counterclockwise(all_patch_vertices_map, all_patch_skeleton_vertices_idx);//resort the boundary vertices.

		//1. construct the SKTMesh.
		//construct the first SKTFace: use the vector<map>: all_patch_skeleton_vertices_idx
		std::set<TSkeleton::Vertex, easy3d::dvec3> skt_vertices_pos;

		std::set<int> been_pushed_patch_idx;
		
		//1.1  init the first SKTFace.
		int id(0);
		std::vector<Point3d> per_face_vertices;
		auto first_patch_skeleton_vertices_idx = all_patch_skeleton_vertices_idx[id];
		for (auto v_ite : first_patch_skeleton_vertices_idx) {
			int patch_sktv_idx = v_ite.first;
			int rootmesh_sktv_idx = v_ite.second;
			auto rootmesh_v = get_mesh_vertex_by_idx(rootmesh_, rootmesh_sktv_idx);
			auto rootmesh_v_pos = static_cast<easy3d::dvec3>(rootmesh_.position(rootmesh_v));
			auto new_point = Point3d(rootmesh_v_pos.x, rootmesh_v_pos.y, rootmesh_v_pos.z);
			per_face_vertices.push_back(new_point);
		}
		add_polygon_face(tsk, per_face_vertices, TSkeleton::Halfedge_handle{});
		been_pushed_patch_idx.insert(0);

		//1.2 int other SKTFace
		do {
			

		} while (been_pushed_patch_idx.size() != meshes_.size());
		
	}


	TSkeleton::Halfedge_handle TSkeletonCreator::add_polygon_face(
		TSkeleton& tsk,
		std::vector<Point3d>& polygon_face_vertices,
		TSkeleton::Halfedge_handle& he_) {
		// assume that all input are valid.

		int n = polygon_face_vertices.size();
		tsk.reserve(
			n + tsk.size_of_vertices(),
			2 * n + tsk.size_of_halfedges(),
			1 + tsk.size_of_facets());

		// 1. use the modifier to create the loop.
		SimplePolygonModifier modifier(polygon_face_vertices, tsk, he_);
		tsk.delegate(modifier);

		TSkeleton::Halfedge_handle he = modifier.get_first_border_halfedge();

		if (he != TSkeleton::Halfedge_handle{}) {
			return tsk.fill_hole(he);
		}
	}

	void TSkeletonCreator::init() {
		
	}



	//================================================= pre compute functions =================================================

	void TSkeletonCreator::pre_compute() {

	}
	void TSkeletonCreator::collect_skeleton_vertices_idx(
		std::vector<std::map<int, int>>& all_patch_vertices_map,
		std::vector<int>& skeleton_vertices_idx,
		std::vector<std::map<int, int>>& all_patch_skeleton_vertices_idx) {
		//0. clear for safe.
		skeleton_vertices_idx.clear();
		all_patch_skeleton_vertices_idx.clear();

		//0. data preparations
		easy3d::SurfaceMesh* rootmesh_ = rootmesh.get();// a deep copy of class var rootmesh.
		std::vector<easy3d::SurfaceMesh*> meshes_(meshes.size());// a deep copy of class var meshes.
		for (int i = 0; i < meshes.size(); i++) {
			meshes_[i] = meshes[i].get();
		}
		//0.1 get all pacthes's boundary vertices's idx and pos, default counterclockwise.
		std::vector<std::vector<int>> all_patch_boundary_vertices_idx;
		std::vector<std::vector<easy3d::dvec3>> all_patch_boundary_vertices_pos;
		for (int i = 0; i < meshes_.size(); i++) {
			std::vector<int> per_patch_boundary_vertices_idx;
			std::vector<easy3d::dvec3> per_patch_boundary_vertices_pos;
			collect_boundary_vertices_counterclockwise(i, per_patch_boundary_vertices_idx, per_patch_boundary_vertices_pos);
			all_patch_boundary_vertices_idx.push_back(per_patch_boundary_vertices_idx);
			all_patch_boundary_vertices_pos.push_back(per_patch_boundary_vertices_pos);
		}

		//1.1 data preparations
		std::vector<std::map<int, int>> all_patch_vertices_map;//patch_vertex_idx -> rootmesh_vertex_idx.
		for (int i = 0; i < meshes_.size(); i++) {//start construct the vertices_map for per valid i.
			//0. data preparations.
			std::map<int, int> per_patch_vertices_map;
			auto per_patch_vertices_idx = all_patch_boundary_vertices_idx[i];
			auto per_patch_vertices_pos = all_patch_boundary_vertices_pos[i];
			int per_patch_size = per_patch_vertices_idx.size();
			//1. construct the per_patch_vertices_map.
			for (int j = 0; j < per_patch_size; j++) {
				auto per_patch_vertex_pos = per_patch_vertices_pos[j];
				auto rootmesh_vertex_idx_by_pos = get_mesh_vertex_idx_by_pos(*rootmesh_, per_patch_vertex_pos);
				per_patch_vertices_map.emplace(per_patch_vertices_idx, rootmesh_vertex_idx_by_pos);
			}
			all_patch_vertices_map.push_back(per_patch_vertices_map);
		}

		//2. evaluate the valence of every vertices on the cut.
		std::map<int, int> cut_vertices_valence;
		cut_vertices_valence.clear();
		for (int i = 0; i < all_patch_vertices_map.size(); i++) {
			auto per_patch_vertices_map = all_patch_vertices_map[i];
			for (int j = 0; j < per_patch_vertices_map.size(); j++) {
				auto it_found = std::find(cut_vertices_valence.begin(), cut_vertices_valence.end(), per_patch_vertices_map[j]);
				if (it_found == cut_vertices_valence.end()) {// the vertex not included in cut_vertices which on the rootmesh.
					cut_vertices_valence.insert(per_patch_vertices_map[j], 0);
				}
				else {// the case that the vertex has been included in the cur_vertices which on the rootmesh.
					//update the existed vertex valence.
					int next_valence = cut_vertices_valence[per_patch_vertices_map[j]];
					next_valence++;
					cut_vertices_valence[per_patch_vertices_map[j]] = next_valence;
				}
			}
		}

		//3. extract all vertices(idx on the rootmesh and idx on per patch.) which valence is greator than 2.
		//	 those vertices are the vertex we called SKTVertex which is_primary is equal to "true".
		//   the vector: SKTVertex_rootmesh_idx is the subset of the set: cut_vertices_valence.
		//3.1 extract all vertice(idx on the rootmesh).
		std::vector<int> SKTVertex_rootmesh_idx;
		for (auto v_ite : cut_vertices_valence) {
			auto v_idx = v_ite.first;
			auto v_valence = v_ite.second;
			if (v_valence > 2) {
				SKTVertex_rootmesh_idx.push_back(v_idx);
			}
		}
		//3.2 extract all vertice(idx on per patch).
		// the var: all_patch_SKTVertex_per_patch_idx store the vertex on the per patch not on the rootmesh.
		std::vector<std::map<int, int>> all_patch_SKTVertex_per_patch_idx_;
		for (int i = 0; i < all_patch_vertices_map.size(); i++) {
			std::map<int, int> per_patch_SKTVertex_per_patch_idx;
			auto per_patch_vertices_map = all_patch_vertices_map[i];
			for (auto v_idx_pair : per_patch_vertices_map) {
				auto v_idx_per_patch = v_idx_pair.first;
				auto v_idx_rootmesh = v_idx_pair.second;
				auto is_SKTVertex = std::find(SKTVertex_rootmesh_idx.begin(), SKTVertex_rootmesh_idx.end(), v_idx_rootmesh);
				if (is_SKTVertex != SKTVertex_rootmesh_idx.end()) {//find the SKTVertex in the per patch.
					per_patch_vertices_map.emplace(v_idx_per_patch, v_idx_rootmesh);
				}
			}
			all_patch_SKTVertex_per_patch_idx_.push_back(per_patch_SKTVertex_per_patch_idx);
		}

		//4. return the result.
		skeleton_vertices_idx = SKTVertex_rootmesh_idx;
		all_patch_skeleton_vertices_idx = all_patch_SKTVertex_per_patch_idx_;
	}
	// another rewrite for the function: collect_skeleton_vertices_idx(...).
	void TSkeletonCreator::collect_skeleton_vertices_idx(
		std::vector<std::map<int, int>>& all_patch_vertices_map,
		std::vector<int>& skeleton_vertices_idx,
		std::vector<std::map<int, int>>& all_patch_skeleton_vertices_idx
	) {
		//1.evaluate the valence of every vertices on the cut.
		std::map<int, int> cut_vertices_valence;
		cut_vertices_valence.clear();
		for (int i = 0; i < all_patch_vertices_map.size(); i++) {
			auto per_patch_vertices_map = all_patch_vertices_map[i];
			for (int j = 0; j < per_patch_vertices_map.size(); j++) {
				auto it_found = std::find(cut_vertices_valence.begin(), cut_vertices_valence.end(), per_patch_vertices_map[j]);
				if (it_found == cut_vertices_valence.end()) {// the vertex not included in cut_vertices which on the rootmesh.
					cut_vertices_valence.insert(per_patch_vertices_map[j], 0);
				}
				else {// the case that the vertex has been included in the cur_vertices which on the rootmesh.
					//update the existed vertex valence.
					int next_valence = cut_vertices_valence[per_patch_vertices_map[j]];
					next_valence++;
					cut_vertices_valence[per_patch_vertices_map[j]] = next_valence;
				}
			}
		}

		//2. extract all vertices(idx on the rootmesh and idx on per patch.) which valence is greator than 2.
		//	 those vertices are the vertex we called SKTVertex which is_primary is equal to "true".
		//   the vector: SKTVertex_rootmesh_idx is the subset of the set: cut_vertices_valence.
		//2.1 extract all vertice(idx on the rootmesh).
		std::vector<int> SKTVertex_rootmesh_idx;
		for (auto v_ite : cut_vertices_valence) {
			auto v_idx = v_ite.first;
			auto v_valence = v_ite.second;
			if (v_valence > 2) {
				SKTVertex_rootmesh_idx.push_back(v_idx);
			}
		}
		//2.2 extract all vertice(idx on per patch).
		// the var: all_patch_SKTVertex_per_patch_idx store the vertex on the per patch not on the rootmesh.
		std::vector<std::map<int, int>> all_patch_SKTVertex_per_patch_idx_;
		for (int i = 0; i < all_patch_vertices_map.size(); i++) {
			std::map<int, int> per_patch_SKTVertex_per_patch_idx;
			auto per_patch_vertices_map = all_patch_vertices_map[i];
			for (auto v_idx_pair : per_patch_vertices_map) {
				auto v_idx_per_patch = v_idx_pair.first;
				auto v_idx_rootmesh = v_idx_pair.second;
				auto is_SKTVertex = std::find(SKTVertex_rootmesh_idx.begin(), SKTVertex_rootmesh_idx.end(), v_idx_rootmesh);
				if (is_SKTVertex != SKTVertex_rootmesh_idx.end()) {//find the SKTVertex in the per patch.
					per_patch_SKTVertex_per_patch_idx.emplace(v_idx_per_patch, v_idx_rootmesh);
				}
			}
			all_patch_SKTVertex_per_patch_idx_.push_back(per_patch_SKTVertex_per_patch_idx);
		}

		//3. return the result.
		skeleton_vertices_idx = SKTVertex_rootmesh_idx;
		all_patch_skeleton_vertices_idx = all_patch_SKTVertex_per_patch_idx_;
	}

	void TSkeletonCreator::extract_all_patch_vertices_map(std::vector<std::map<int, int>>& all_patch_vertices_map) {
		//0. clear for safe.
		all_patch_vertices_map.clear();

		//1. data preparations
		easy3d::SurfaceMesh* rootmesh_ = rootmesh.get();// a deep copy of class var rootmesh.
		std::vector<easy3d::SurfaceMesh*> meshes_(meshes.size());// a deep copy of class var meshes.
		for (int i = 0; i < meshes.size(); i++) {
			meshes_[i] = meshes[i].get();
		}
		//1.1 get all pacthes's boundary vertices's idx and pos, default counterclockwise.
		std::vector<std::vector<int>> all_patch_boundary_vertices_idx;
		std::vector<std::vector<easy3d::dvec3>> all_patch_boundary_vertices_pos;
		for (int i = 0; i < meshes_.size(); i++) {
			std::vector<int> per_patch_boundary_vertices_idx;
			std::vector<easy3d::dvec3> per_patch_boundary_vertices_pos;
			collect_boundary_vertices_counterclockwise(i, per_patch_boundary_vertices_idx, per_patch_boundary_vertices_pos);
			all_patch_boundary_vertices_idx.push_back(per_patch_boundary_vertices_idx);
			all_patch_boundary_vertices_pos.push_back(per_patch_boundary_vertices_pos);
		}

		//2. 
		//2.1 data preparations
		std::vector<std::map<int, int>> all_patch_vertices_map;//patch_vertex_idx -> rootmesh_vertex_idx.
		for (int i = 0; i < meshes_.size(); i++) {//start construct the vertices_map for per valid i.
			//0. data preparations.
			std::map<int, int> per_patch_vertices_map;
			auto per_patch_vertices_idx = all_patch_boundary_vertices_idx[i];
			auto per_patch_vertices_pos = all_patch_boundary_vertices_pos[i];
			int per_patch_size = per_patch_vertices_idx.size();
			//1. construct the per_patch_vertices_map.
			for (int j = 0; j < per_patch_size; j++) {
				auto per_patch_vertex_pos = per_patch_vertices_pos[j];
				auto rootmesh_vertex_idx_by_pos = get_mesh_vertex_idx_by_pos(*rootmesh_, per_patch_vertex_pos);
				per_patch_vertices_map.emplace(per_patch_vertices_idx, rootmesh_vertex_idx_by_pos);
			}
			all_patch_vertices_map.push_back(per_patch_vertices_map);
		}
	}

	void TSkeletonCreator::collect_boundary_vertices_counterclockwise(
		int& mesh_idx,
		std::vector<int>& boundary_vertices_idx_counterclockwise,
		std::vector<easy3d::dvec3>& boundary_vertices_pos_counterclockwise) {
		//0. data clear.
		boundary_vertices_idx_counterclockwise.clear();
		boundary_vertices_pos_counterclockwise.clear();

		// assume that input: mesh_idx is valid.
		//0. var preparations
		auto mesh = meshes[mesh_idx].get();//the deep copy of target mesh(by valid mesh_idx)

		//1. find the first boundary vertex idx
		easy3d::SurfaceMesh::Vertex first_v, curr_v;
		bool found_first = false;
		for (auto v : mesh->vertices()) {
			if (mesh->is_border(v)) {//determine whether vertex:v is a boundary vertex.
				first_v = v;
				curr_v = v;
				found_first = true;
				break;
			}
		}

		//2.collect all boundary vertices which the head is first_v
		std::vector<int> been_pushed;
		been_pushed.push_back(first_v.idx());
		boundary_vertices_idx_counterclockwise.push_back(curr_v.idx());
		easy3d::dvec3 pos = static_cast<easy3d::dvec3>(mesh->position(curr_v));
		boundary_vertices_pos_counterclockwise.push_back(pos);

		do {
			bool found_next = false;
			easy3d::SurfaceMesh::Halfedge next_he;

			for (auto he : mesh->halfedges(curr_v)) {
				if (mesh->is_border(he)) {
					auto target_v = mesh->target(he);
					auto it_v = std::find(been_pushed.begin(), been_pushed.end(), target_v.idx());
					if (it_v == been_pushed.end()) {
						been_pushed.push_back(target_v.idx());
						boundary_vertices_idx_counterclockwise.push_back(target_v.idx());
						easy3d::dvec3 pos_ = static_cast<easy3d::dvec3>(mesh->position(target_v));
						boundary_vertices_pos_counterclockwise.push_back(pos_);
						curr_v = target_v;
						found_next = true;
						break;
					}
				}
			}
		} while (curr_v != first_v);
	}
	
	//======================================= resort functions ======================================

	void TSkeletonCreator::resort_boundary_vertices_counterclockwise(int& mesh_idx, int& skt_vertex_idx,
		std::vector<int>& boundary_vertices_idx_counterclockwise,
		std::vector<int>& boundary_vertices_pos_counterclockwise) {
		// assume that all input are valid.

		auto ite = std::find(boundary_vertices_idx_counterclockwise.begin(), boundary_vertices_idx_counterclockwise.end(), skt_vertex_idx);
		if (ite != boundary_vertices_idx_counterclockwise.end()) {
			std::rotate(boundary_vertices_idx_counterclockwise.begin(), ite, boundary_vertices_idx_counterclockwise.end());
			int i = std::distance(boundary_vertices_idx_counterclockwise.begin(), ite);
			int i_ = 0;
			for (auto it = boundary_vertices_pos_counterclockwise.begin(); it != boundary_vertices_idx_counterclockwise.end(); it++) {
				if (i_ == i) {//find the it in boundary_vertices_pos_counterclockwise.
					std::rotate(boundary_vertices_pos_counterclockwise.begin(), it, boundary_vertices_pos_counterclockwise.end());
					break;
				}
				i_++;
			}

		}
		else {
			std::cout << "[TSkeleton::resort_boundary_vertices_idx_counterclockwise]: input skt_vertex_idx is invalid." << std::endl;
		}
	}
	/**
	 * 
	 */
	void TSkeletonCreator::resort_boundary_vertices_counterclockwise(
		std::vector<std::map<int, int>>& all_patch_vertices_map,
		std::vector<std::map<int, int>>& all_patch_skeleton_vertices_idx) {
		// assume that all input are valid.
		// 0. do for every patch.
		for (size_t patch_idx = 0; patch_idx < all_patch_vertices_map.size(); ++patch_idx) {
			auto& patch_vertices_map = all_patch_vertices_map[patch_idx];
			auto& patch_skeleton_vertices_idx = all_patch_skeleton_vertices_idx[patch_idx];

			// if the patch_skeleton_vertices_idx is empty，skip.(if the input is valid, then this case will not happen.)
			if (patch_skeleton_vertices_idx.empty()) {
				continue;
			}

			// 1. get the first skeleton vertex(idx)
			auto first_skeleton_it = patch_skeleton_vertices_idx.begin();
			int first_skeleton_patch_idx = first_skeleton_it->first;

			// 2. store the original patch_skeleton_vertices_idx backup which will be used for the search later.
			std::map<int, int> original_skeleton_vertices_idx = patch_skeleton_vertices_idx;

			// 3. transform the map to vector which is available for the operation: rotate.
			std::vector<std::pair<int, int>> vertices_vec(patch_vertices_map.begin(), patch_vertices_map.end());

			// 4. find the position of the first skeleton vertex in vertices_vec
			auto skeleton_vertex_it = std::find_if(vertices_vec.begin(), vertices_vec.end(),
				[first_skeleton_patch_idx](const std::pair<int, int>& p) {
					return p.first == first_skeleton_patch_idx;
				});

			// 5. if the skeleton vertex is found, perform rotation
			if (skeleton_vertex_it != vertices_vec.end()) {
				// use std::rotate to rotate the skeleton vertex to the beginning
				std::rotate(vertices_vec.begin(), skeleton_vertex_it, vertices_vec.end());
			}
			else {
				std::cout << "[TSkeletonCreator::resort_boundary_vertices_counterclockwise]: skeleton vertex not found in patch " 
					<< patch_idx << std::endl;
				continue;
			}

			// 6. rebuild patch_vertices_map (map will automatically sort by key, but we have maintained the correct order)
			patch_vertices_map.clear();
			patch_vertices_map.insert(vertices_vec.begin(), vertices_vec.end());

			// 7. reorder all_patch_skeleton_vertices_idx according to the order of skeleton_vertex appearance in the rotated all_patch_vertices_map
			//    traverse the rotated vertices_vec, find all skeleton vertices, and rebuild skeleton_vertices_idx in the order of appearance
			std::vector<std::pair<int, int>> resorted_skeleton_vec;
			for (const auto& vertex_pair : vertices_vec) {
				int patch_vertex_idx = vertex_pair.first;
				auto skeleton_it = original_skeleton_vertices_idx.find(patch_vertex_idx);
				if (skeleton_it != original_skeleton_vertices_idx.end()) {
					resorted_skeleton_vec.push_back(std::make_pair(skeleton_it->first, skeleton_it->second));
				}
			}

			// 8. rebuild patch_skeleton_vertices_idx
			patch_skeleton_vertices_idx.clear();
			patch_skeleton_vertices_idx.insert(resorted_skeleton_vec.begin(), resorted_skeleton_vec.end());
		}
	}



	//================================================= inline functions =================================================
	inline easy3d::SurfaceMesh::Vertex TSkeletonCreator::get_mesh_vertex_by_idx(int& mesh_idx, int vertex_idx)const {
		// assume that input vertex_idx is valid.
		auto mesh = meshes[vertex_idx].get();
		int curr_vertex_idx(0);
		for (auto v : mesh->vertices()) {
			if (curr_vertex_idx == vertex_idx) return v;
			curr_vertex_idx++;
		}
		return easy3d::SurfaceMesh::Vertex{};
	}
	inline easy3d::SurfaceMesh::Vertex TSkeletonCreator::get_mesh_vertex_by_idx(easy3d::SurfaceMesh& mesh, int vertex_idx)const {
		// assume that all input are valid.
		int curr_vertex_idx(0);
		for (auto v : mesh.vertices()) {
			if (curr_vertex_idx == vertex_idx) return v;
			curr_vertex_idx++;
		}
		return easy3d::SurfaceMesh::Vertex{};
	}
	inline int TSkeletonCreator::get_mesh_vertex_idx_by_pos(easy3d::SurfaceMesh& mesh, easy3d::dvec3& pos)const {
		// assume that all input are valid.
		for (auto v : mesh.vertices()) {
			auto v_pos_dvec3 = static_cast<easy3d::dvec3>(mesh.position(v));
			if (pos == v_pos_dvec3) {// if the operator:== is invalid, then write a new inline functionn to deal it.
				return v.idx();
			}
		}
		return -1;
	}

	inline int TSkeletonCreator::get_corresponding_vertex_idx(std::map<int, int>& vertices_map, int& idx)const {
		// assume that all input are valid.
		auto ite = std::find(vertices_map.begin(), vertices_map.end(), idx);
		if (ite != vertices_map.end()) {
			return ite->second;
		}
		else
		{
			return -1;
		}
	}

	inline void TSkeletonCreator::deepcopy(easy3d::SurfaceMesh& rootmesh_)const {
		auto rm = rootmesh.get();
		rootmesh_ = *rm;
	}
	inline void TSkeletonCreator::deepcopy(std::vector<easy3d::SurfaceMesh>& meshes_)const {
		meshes_.clear();
		for (int i = 0; i < meshes.size(); i++) {
			auto p = meshes[i].get();
			meshes_.push_back(*p);
		}
	}

}

