#pragma once
#include <easy3d/viewer/viewer.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/point_cloud.h>

#include <lib/Easy3D/3rd_party/imgui/imgui.h>

#include "Tspline.h"
#include "TsplineCreator.h"


struct ImGuiContext;

class MyImGui :public easy3d::Viewer {
public:
	explicit MyImGui(
		const std::string& title = "SimpleIMGui",
		int samples = 4,
		int gl_major = 3,
		int gl_minor = 2,
		bool full_screen = false,
		bool resizable = true,
		int depth_bits = 24,
		int stencil_bits = 8,
		int width = 1200,
		int height = 800
	);
	~MyImGui()override;

protected:
	void init()override;
	void pre_draw()override;
	void post_draw()override;
	void post_resize(int w, int h)override;

	bool callback_event_cursor_pos(double x, double y)override;
	bool callback_event_mouse_button(int button, int action, int modifiers)override;
	bool callback_event_keyboard(int key, int action, int modifiers)override;
	bool callback_event_character(unsigned int codepoint) override;
	bool callback_event_scroll(double dx, double dy) override;

	void draw_custom_gui();
	void draw_main_menu();
	void draw_info_panel();
	void draw_control_panel();
	void draw_control_panel_1();//designed for the total model.

	//
	void clear_all();
	void open_model_file();
	void load_obj_file();

	void next_mesh();
	void last_mesh();

	void load_param();
	void next_param();
	void last_param();

	void next_tmesh();
	void last_tmesh();

	void load_result();
	void load_result_quad();


	void save_model();
	void save_model(const std::string& filename);

	void set_renderers(std::shared_ptr<easy3d::SurfaceMesh>& model);
	//============================================== precompute functions ===================================
	void load_txt_file(const std::string& filename);
	//void compute_Skeleton(easy3d::SurfaceMesh& rootmesh, std::vector<easy3d::SurfaceMesh>& meshes);

	//=======================================================================================================
	void Parameterization_uniform_weight();
	void Parameterization_chord_weight();

	void compute_control_points(tspline::Tspline& tsp, std::vector<easy3d::dvec3>& mesh, std::vector<easy3d::dvec3>& mesh_param);
	void compute_orignal_tmesh(const int segX,const int segY);
	void compute_result_mesh(tspline::Tspline& tsp, easy3d::SurfaceMesh* mesh_param);
	void compute_result_quad_mesh(tspline::Tspline& tsp, unsigned& segX, unsigned& segY);//compute the result quad mesh.

	void convert_vec3_2_dvec3(easy3d::SurfaceMesh* mesh, std::vector<easy3d::dvec3>& param_dvec3);
	void convert_tsp_2_surfacemesh(const tspline::Tspline& tsp, easy3d::SurfaceMesh& mesh);
	void debug_tmesh_info(const tspline::Tspline& tsp, const easy3d::SurfaceMesh& mesh_tsp);

	void Global_LSM(double& epsilon, unsigned& min_point_count);

	//=========================================================== auxiliary functions
	void evaluate_single_tface_error(tspline::Tspline& tsp,
		tspline::Tspline::Face_iterator& fit,
		std::vector<easy3d::dvec3>& mesh,
		std::vector<easy3d::dvec3>& mesh_param,
		unsigned min_point_count,
		double& face_error);
	void evaluate_all_tface_error(tspline::Tspline& tsp,
		std::vector<easy3d::dvec3>& mesh, 
		std::vector<easy3d::dvec3>& mesh_param,
		unsigned min_point_count,
		std::vector<double>& all_face_error);
	inline void evaluate_dis(Eigen::Vector3d& p1, Eigen::Vector3d& p2, double& result);
	inline void get_params_in_face(
		tspline::Tspline& tsp,
		tspline::Tspline::Face_iterator& fit,
		std::vector<easy3d::dvec3>& params,
		std::vector<int>& params_in_face);
	inline void update_n_vertex_in_tfaces(tspline::Tspline& tsp, std::vector<int>& n_param);
	inline void update_n_vertex_in_tfaces(tspline::Tspline& tsp, std::vector<easy3d::dvec3>& param_double);
	inline void update_all_tface_error(tspline::Tspline& tsp, std::vector<double>& all_tface_error);
	inline easy3d::SurfaceMesh::Face get_mesh_face_by_idx(easy3d::SurfaceMesh* mesh, int face_idx)const;
	inline easy3d::SurfaceMesh::Vertex get_mesh_vertex_by_idx(easy3d::SurfaceMesh* mesh, int vertex_idx)const;

protected:
	/*
		GUI vars
	*/
	static ImGuiContext* context_;

	bool show_control_panel_ = true;
	bool show_info_panel_ = true;

	float menu_height_ = 0.0f;

	void setup_imgui_style();
	void reload_font(int font_size = 16);
	float pixel_ratio();

protected:

	bool isLoadOBJ;
	bool isParam;
	bool isComputeTMesh;

	int mesh_idx;
	int mesh_tsp_idx;

	//the var root_meshes and rootmeshes_patches are designed for the control panel 1.
	std::shared_ptr<easy3d::SurfaceMesh> rootmesh;
	std::vector<int> rootmesh_patches;

	std::vector<std::shared_ptr<easy3d::SurfaceMesh>> meshes;
	std::vector<std::shared_ptr<easy3d::SurfaceMesh>> mesh_params;
	std::vector<std::vector<std::shared_ptr<tspline::Tspline>>> mesh_tsps;
	std::vector<std::vector<std::shared_ptr<easy3d::SurfaceMesh>>> mesh_results;
	std::vector<std::vector<std::shared_ptr<easy3d::SurfaceMesh>>> mesh_results_quad;


};

