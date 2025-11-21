#pragma once
#include "TSkeletonTypes.h"
#include "Tspline.h"
#include "TsplineCreator.h"
#include "MyParam.h"
#include <CGAL/Modifier_base.h>

namespace skeleton {
	/**
	 * @brief main skeleton class for 3D closed mesh design
	 *        similar to tspline::Tspline but for 3D space using Polyhedron_3
	 *        inherits from SkeletonPolyhedron to leverage CGAL's polyhedron functionality
	 */
	class TSkeleton : public SkeletonPolyhedron {
	public:
		TSkeleton() : SkeletonPolyhedron() {}


	public:
		void refine_global_LSM(unsigned& min_point_count, double& epsilon);
		void compute_local_param(int& vid);
		void compute_local_tsp(int& vid);
		void merge_tsplines(tspline::Tspline& tsp1, tspline::Tspline& tsp2);
		void merge_params(easy3d::SurfaceMesh& param1, easy3d::SurfaceMesh& param2);
		void adjust(tspline::Tspline& tsp1, double& lambda);//may not use.
		void adjust(easy3d::SurfaceMesh& mesh1, double& lambda);//may not use.

		// ================================== construct functions =======================================
		TSkeleton::Vertex_handle add_vertex_to_skeleton(const Point3d& pt);
		TSkeleton::Halfedge_handle add_triangle_face_to_skeleton(std::vector<TSkeleton::Vertex_handle>& vhs);

		// ================================== compatible check ==========================================
		bool check_congruent(std::vector<TSkeleton::Face_handle>& per_facet_vertices_valence);

		// ================================== =========================================
		/**
		 * 
		 * .
		 */
		void init(int& start_fid);

		void init_base_he(int& fid, int& cv1, int& cv2, double& d_geodestic, double& d_param);
		void init_opposite_he_by_he(double& d_geodestic, double& d_param, int& fid, int& cv1, int& cv2);
		void init_adjacent_he_by_he(double& d_geodestic, double& d_param, int& fid, int& cv1, int& cv2, double& re_d_geodestic, double& re_d_param);
		void init_adjacent_he_by_he(double& d_geodestic, double& d_param, int& fid, int& cv1, int& cv2);
		void init_corner_vertices_id_by_fid_hid(int& fid, int& hid, int& cv1, int& cv2);

		void collect_he_skeleton_vertices_idx(int& fid, int& cv1, int& cv2, std::vector<int>& vertices);
		

		TSkeleton::Face_handle get_start_face();
		int get_start_face_idx();

		// ================================== =========================================
		double compute_gedestic_distance(easy3d::SurfaceMesh& mesh, std::vector<int>& vertices);
		std::vector<int> compute_facet_valence(int& fh_count);
		std::vector<double> compute_skeleton_vertices_d_geodestic(easy3d::SurfaceMesh& mesh, TSkeleton::Face_handle& fh);
		double compute_halfedge_d_geodestic(easy3d::SurfaceMesh& mesh, TSkeleton::Halfedge_handle& he);

		
		std::set<int> collect_adjacent_facet_idx(int& fid, std::set<int>& been_init_facet);

		double compute_consistency_d_param(TSkeleton::Halfedge_handle& dual_he);

		//=================================== inline functions ===================================
		inline TSkeleton::Vertex_handle get_vh_by_id(int& id)const;
		inline void collect_facet_vertices_idx(int& f_id, std::vector<int>& facet_vertices);
		inline void collect_facet_corner_vertices_idx(int& f_id, std::vector<int>& facet_corner_vertices);
		inline void collect_facet_halfedges_id(int& fid, std::vector<int>& vertices, std::vector<int>& he_ids);


	public:
		bool quiet;
		bool clamped;
		unsigned degree;

		std::shared_ptr<easy3d::SurfaceMesh> rootmesh;// input
		std::vector<std::shared_ptr<easy3d::SurfaceMesh>> meshes;// input

		std::vector<std::shared_ptr<easy3d::SurfaceMesh>> meshes_param;
		std::vector<std::shared_ptr<easy3d::SurfaceMesh>> meshes_tsp;

		int root_patch_idx;


		std::shared_ptr<easy3d::SurfaceMesh> tsk_tsp_easy3d;
		std::shared_ptr<tspline::Tspline> tsk_tsp;
	};

	//====================================================================================================================

	class TSkeletonCreator {
	public:
		TSkeletonCreator(
			easy3d::SurfaceMesh* rootmesh,
			std::vector<easy3d::SurfaceMesh*>& meshes,
			TSkeleton& tsk);
		~TSkeletonCreator() {}
	public:
		std::shared_ptr<easy3d::SurfaceMesh> rootmesh;
		std::vector<std::shared_ptr<easy3d::SurfaceMesh>> meshes;
	public:
		//================================================= construct functions ========================================
		// ================================ construct functions ========================================
		/**
		 * TODO:
		 * 1.use the SKTVertex(which var: is_primary is true) to construct the SKTMesh.(get the counterclockwise mesh.)
		 * 2.bind the SKTFace(per patch) with an SKTVertex which its var: is_primary is true, bijection.(set the SKTVertex var: patch_idx)
		 * 3.second resort the boundary_vertices_idx_ccounterclockwise, makesure that the head(idx=0) is the left bottom SKTVertex
		 * （不计算参数化结果）
		 * details in 1. construct the SKTMesh.
		 * 1.获取所有SKTVertex(true);
		 * 2.获取每个patch(对应着meshes[idx],idx有效)的所有边界顶点（逆时针排序）;
		 * 3.根据逆时针排序的patch边界顶点构建逆时针方向的SKTMesh;
		 * 4.经过3，SKTMesh已经有了逆时针的网格结构，为每个SKTVertex(true)绑定一个唯一的patch（同上），并且每一个patch对应唯一一个SKTVertrx(true).
		 *
		 * details:(已经获得了在rootmesh上的所有SKTVertex的idx，在每一个patch上SKTVertex对应的索引)
		 *		1.从某一个指定patch开始，根据其上的SKTVertex的idx寻找到rootmesh上的SKTVertex,创建SKTVertex对象并初始化，因为该patch的边界点是
		 *		按照逆时针方向排序的，因此可以根据此构建一个SKTFace，该SKTFace将作为BFS数的根节点;
		 *		2.构建SKTFace的BFS树（同等于patch的BFS树），树的根节点为1.中得到的SKTFace，将其标记为已经添加，随后寻找与该SKTFace对应的patch
		 *		相邻的所有patch（利用patch的边界点集合是否有公共边，或者说如果两个patch有公共边，那么在rootmesh上其一定共享至少一个SKTVertex，
		 *		于是寻找到在相邻patch上的SKTVertex(其空间坐标pos和其父节点的SKTVertex的pos一致（完全相等），于是只需要确定两个patch之间是否有
		 *		公共的SKTVertex(在rootmesh上)，就可以构建与父SKTFace相邻的所有SKTFace.)）
		 * .
		 */
		void construct(TSkeleton& tsk);
		


		void construct_new(TSkeleton& tsk);

		/**
		 * 在调用construct()函数后调用。
		 * TODO:
		 * （计算参数化结果）
		 * details:
		 * 1.选定一个SKTVertex(true),对其绑定的patch进行选定参数化计算（参数化方法由前台交互得到）;
		 * 2.构建BFS树(SKTFace的BFS树)，为SKTMesh中每一个patch计算参数化;参数化必须保证在公共边处的顶点分配一致,规定缩放因子恒为1;
		 * 3.根据先前所锚定的参数化边界点分布来决定后续patch的参数化结果边界点分布;
		 * 4.利用BFS树拼接每一个Patch的TMesh,于是得到平移量s(定义在SKTHalfedge中);
		 * 5.第三步重复直到所有SKTVertex都有合理的节点向量。
		 * .
		 */
		void init();

		TSkeleton::Halfedge_handle add_polygon_face(
			TSkeleton& tsk, 
			std::vector<Point3d>& polygon_face_vertices,
			TSkeleton::Halfedge_handle& he_);

		 // ================================ pre compute functions =====================
		void pre_compute();
		void collect_skeleton_vertices_idx(
			std::vector<std::map<int, int>>& all_patch_vertices_map,
			std::vector<int>& skeleton_vertices_idx,
			std::vector<std::map<int, int>>& all_patch_skeleton_vertices_idx);
		void collect_skeleton_vertices_idx(
			std::vector<std::map<int, int>>& all_vertices_map,
			std::vector<int>& skeleton_vertices_idx,
			std::vector<std::map<int, int>>& all_patch_skeleton_vertices_idx,
			std::vector<std::map<int, easy3d::dvec3>>& all_patch_skeleton_vertices_pos);
		//extract the vertices map: per patch's boundary vertices -> rootmesh's cut graph vertices which contains all the SKTVertex.
		void extract_all_patch_vertices_map(std::vector<std::map<int, int>>& all_patch_vertices_map);
		/**
		 * collect boundary vertices, which will be used in the function: construct_SKTMesh.
		 * the head of boundary_vertices_idx_counterclockwise may not the left bottom SKTVertex.
		 * .
		 */
		void collect_boundary_vertices_counterclockwise(
			int& mesh_idx,
			std::vector<int>& boundary_vertices_idx_counterclockwise,
			std::vector<easy3d::dvec3>& boundary_vertices_pos_counterclockwise);
		/**
		 * resort the boundary_vertices_idx_counterclockwise which the head(idx=0) of boundary_* is skt_vertex_idx.
		 * .
		 */
		void resort_boundary_vertices_counterclockwise(
			std::vector<std::map<int, int>>& all_patch_vertices_map,
			std::vector<std::map<int, int>>& all_patch_skeleton_vertices_idx);
		void resort_boundary_vertices_counterclockwise(
			std::map<int, int>& one_patch_vertices_map,
			std::map<int, int>& one_patch_skeleton_vertices_idx);
		void resort_boundary_vertices_counterclockwise(
			int& mesh_idx, int& skt_vertex_idx,
			std::vector<int>& boundary_vertices_idx_counterclockwise,
			std::vector<int>& boundary_vertices_pos_counterclockwise);

		// =============================================== inline functions =======================================

		inline easy3d::SurfaceMesh::Vertex get_mesh_vertex_by_idx(int& mesh_idx, int vertex_idx)const;
		inline easy3d::SurfaceMesh::Vertex get_mesh_vertex_by_idx(easy3d::SurfaceMesh& mesh, int vertex_idx)const;
		//inline std::vector<int> get_mesh_vertex_id_by_pos(easy3d::SurfaceMesh& mesh, std::vector<easy3d::dvec3>& pos)const;
		inline int get_mesh_vertex_idx_by_pos(easy3d::SurfaceMesh& mesh, easy3d::dvec3& pos)const;

		inline int get_corresponding_vertex_idx(std::map<int, int>& vertices_map, int& idx)const;



		inline void deepcopy(easy3d::SurfaceMesh& rootmesh_)const;
		inline void deepcopy(std::vector<easy3d::SurfaceMesh>& meshes_)const;
	};


	// ============================================= TSkeleton Face Operator Class =============================================
	class PolygonFaceModifier :public CGAL::Modifier_base<TSkeleton::HalfedgeDS> {
	public:
		PolygonFaceModifier(const std::vector<Point3d>& points) :points_(points), first_border_halfedge_() {}
		
		void operator()(TSkeleton::HalfedgeDS& hds) {
			typedef TSkeleton::HalfedgeDS HDS;
			typedef CGAL::HalfedgeDS_decorator<HDS> Decorator;
			typedef typename HDS::Vertex Vertex;
			typedef typename HDS::Halfedge Halfedge;
			typedef typename Halfedge::Base HBase;
		}
	private:
		const std::vector<Point3d>& points_;
		TSkeleton::Halfedge_handle first_border_halfedge_;
	};

}
