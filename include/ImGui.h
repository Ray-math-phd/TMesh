#pragma once
#include <easy3d/viewer/viewer.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/point_cloud.h>

#include <lib/Easy3D/3rd_party/imgui/imgui.h>

#include "Tspline.h"


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

	//
	void clear_all();
	void open_model_file();
	void load_obj_file();

	void next_mesh();
	void last_mesh();

	void next_param();
	void last_param();

	void next_tmesh();
	void last_tmesh();

	void next_result();
	void last_result();

	void save_model();
	void save_model(const std::string& filename);



	//==============================================
	void Parameterization_uniform_weight();
	void Parameterization_chord_weight();

	void compute_control_points(tspline::Tspline& tsp, std::vector<easy3d::dvec3>& param, std::vector<easy3d::dvec3>& patch);
	void compute_orignal_tmesh();

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

	std::vector<std::shared_ptr<easy3d::SurfaceMesh>> meshes;
	std::vector<std::shared_ptr<easy3d::SurfaceMesh>> mesh_params;
	std::vector<std::vector<std::shared_ptr<tspline::Tspline>>> mesh_tsps;
	std::vector<std::vector<std::shared_ptr<tspline::Tspline>>> mesh_results;


};

