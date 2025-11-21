#include "MyImGui.h"
#include "MyParam.h"

#include <algorithm>
#include <charconv>
#include <map>
#include <iterator>
#include <set>
#include <memory>

#include <easy3d/util/file_system.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/state.h>
#include <easy3d/renderer/buffer.h>

#include <easy3d/util/dialog.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/file_system.h>
#include <easy3d/fileio/surface_mesh_io.h>

#include <lib/Easy3D/3rd_party/imgui/imgui.h>
#include <lib/Easy3D/3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>
#include <lib/Easy3D/3rd_party/glfw/include/GLFW/glfw3.h>
#include <lib/Easy3D/3rd_party/imgui/backends/imgui_impl_glfw.h>
#include <lib/Easy3D/3rd_party/imgui/backends/imgui_impl_opengl3.h>

ImGuiContext* MyImGui::context_ = nullptr;

MyImGui::MyImGui(
	const std::string& title,
	int samples,
	int gl_major,
	int gl_minor,
	bool full_screen,
	bool resizable,
	int depth_bits,
	int stencil_bits,
	int width,
	int height
) :easy3d::Viewer(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits, width, height) {
#if defined(_WIN32)&&defined(_MSC_VER)
	glfwInit();
#endif
	isLoadOBJ = false;
	isParam = false;
	isComputeTMesh = false;

	mesh_idx = -1;

	meshes.clear();
	mesh_params.clear();
	mesh_tsps.clear();
	mesh_results.clear();
	mesh_results_quad.clear();
}

MyImGui::~MyImGui() {
	if (context_) {
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext(context_);
		context_ = nullptr;
	}
}

void MyImGui::init() {
	easy3d::Viewer::init();

	if (!context_) {
		IMGUI_CHECKVERSION();
		context_ = ImGui::CreateContext();

		// 设置ImGui上下文
		ImGui::SetCurrentContext(context_);

		// 初始化ImGui后端
		ImGui_ImplGlfw_InitForOpenGL(window_, true);
		const char* glsl_version = "#version 150";
		ImGui_ImplOpenGL3_Init(glsl_version);

		// 配置ImGui
		ImGuiIO& io = ImGui::GetIO();
		io.WantCaptureKeyboard = true;
		io.WantTextInput = true;
		io.IniFilename = nullptr;

		// 设置样式
		setup_imgui_style();

		// 加载字体
		reload_font();
	}
}

void MyImGui::setup_imgui_style() {
	ImGui::StyleColorsDark();

	ImGuiStyle& style = ImGui::GetStyle();
	style.FrameRounding = 5.0f;
	style.GrabRounding = 5.0f;
	style.WindowRounding = 5.0f;
	style.PopupRounding = 5.0f;
	style.ScrollbarRounding = 5.0f;
	style.TabRounding = 5.0f;

	style.Colors[ImGuiCol_Header] = ImVec4(0.2f, 0.4f, 0.8f, 0.8f);
	style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.3f, 0.5f, 0.9f, 0.8f);
	style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.1f, 0.3f, 0.7f, 0.8f);
}
void MyImGui::reload_font(int font_size) {
	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->Clear();
	io.Fonts->AddFontFromMemoryCompressedTTF(
		droid_sans_compressed_data,
		droid_sans_compressed_size,
		static_cast<float>(font_size) * dpi_scaling()
	);
	io.FontGlobalScale = 1.0f / pixel_ratio();
	ImGui_ImplOpenGL3_DestroyDeviceObjects();
}

float MyImGui::pixel_ratio() {
	int fw = framebuffer_width();
	int vw = width();
	return static_cast<float>(fw) / static_cast<float>(vw);
}

void MyImGui::post_resize(int w, int h) {
	easy3d::Viewer::post_resize(w, h);
	if (context_) {
		ImGui::GetIO().DisplaySize.x = static_cast<float>(w);
		ImGui::GetIO().DisplaySize.y = static_cast<float>(h);
	}
}

bool MyImGui::callback_event_cursor_pos(double x, double y) {
	if (ImGui::GetIO().WantCaptureMouse) {
		return true;
	}
	else {
		return easy3d::Viewer::callback_event_cursor_pos(x, y);
	}
}

bool MyImGui::callback_event_mouse_button(int button, int action, int modifiers) {
	if (ImGui::GetIO().WantCaptureMouse) {
		return true;
	}
	//	bool caps_lock_pressed = ImGui::IsKeyPressed(ImGuiKey_CapsLock);
	bool caps_lock_down = ImGui::IsKeyDown(ImGuiKey_CapsLock);
	bool caps_lock_released = ImGui::IsKeyReleased(ImGuiKey_CapsLock);
	if (caps_lock_down) {
		easy3d::dialog::message(
			"warning",
			"you have pressed CAPSLOACK, if you want to select Face/Edge/Vertex, please ignore this warning.",
			easy3d::dialog::Choice::ok,
			easy3d::dialog::Type::warning
		);
		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
			double x, y;
			glfwGetCursorPos(window_, &x, &y);

			return true;
		}
	}

	return easy3d::Viewer::callback_event_mouse_button(button, action, modifiers);
}

bool MyImGui::callback_event_keyboard(int key, int action, int modifiers) {
	if (ImGui::GetIO().WantCaptureKeyboard) {
		return true;
	}
	else {
		return easy3d::Viewer::callback_event_keyboard(key, action, modifiers);
	}
}

bool MyImGui::callback_event_character(unsigned int codepoint) {
	if (ImGui::GetIO().WantCaptureKeyboard) {
		return true;
	}
	else {
		easy3d::Viewer::callback_event_character(codepoint);
	}
}

bool MyImGui::callback_event_scroll(double dx, double dy) {
	if (ImGui::GetIO().WantCaptureMouse) {
		return true;
	}
	else {
		return easy3d::Viewer::callback_event_scroll(dx, dy);
	}
}

void MyImGui::pre_draw() {
	// 确保使用正确的ImGui上下文
	ImGui::SetCurrentContext(context_);
	
	ImGui_ImplGlfw_NewFrame();
	ImGui_ImplOpenGL3_NewFrame();
	ImGui::NewFrame();

	easy3d::Viewer::pre_draw();
}

//
void MyImGui::post_draw() {
	// 确保使用正确的ImGui上下文
	ImGui::SetCurrentContext(context_);
	
	// 绘制自定义GUI
	draw_custom_gui();

	// 渲染ImGui
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	easy3d::Viewer::post_draw();
}

//paint my Gui
void MyImGui::draw_custom_gui() {
	//main menu
	draw_main_menu();

	//control menu
	if (show_control_panel_) {
		draw_control_panel();
	}

	//info panel
	if (show_info_panel_) {
		draw_info_panel();
	}
}

void MyImGui::draw_main_menu() {
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
	
	if (ImGui::BeginMainMenuBar()) {
		// File menu
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("Open Model", "Ctrl+O")) {
				open_model_file();
			}
			if (ImGui::MenuItem("Save Model", "Ctrl+S")) {
				save_model();
			}
			if (ImGui::MenuItem("Clear All")) {
				clear_all();
			}
			ImGui::Separator();
			if (ImGui::MenuItem("Exit", "Ctrl+F4")) {
				exit();
			}
			ImGui::EndMenu();
		}

		// View menu
		if (ImGui::BeginMenu("View")) {
			ImGui::MenuItem("Control Panel", nullptr, &show_control_panel_);
			ImGui::MenuItem("Info Panel", nullptr, &show_info_panel_);
			ImGui::EndMenu();
		}

		// Tools menu
		if (ImGui::BeginMenu("Tools")) {
			if (ImGui::BeginMenu("Parameterization")) {
				if (ImGui::MenuItem("BFF", nullptr)) {
					// TODO: implement BFF parameterization
				}
				if (ImGui::MenuItem("Uniform Weight Harmonic", nullptr)) {
					Parameterization_uniform_weight();
					isParam = true;
				}
				if (ImGui::MenuItem("Chord Weight Harmonic", nullptr)) {
					Parameterization_chord_weight();
					isParam = true;
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Compute Control Mesh")) {
				if (ImGui::MenuItem("Compute Control Mesh", nullptr)) {
					static int segX = 4, segY = 4;
					compute_orignal_tmesh(segX, segY);
				}
				ImGui::EndMenu();
			}
			ImGui::EndMenu();
		}

		menu_height_ = ImGui::GetWindowHeight();
		ImGui::EndMainMenuBar();
	}
	
	ImGui::PopStyleVar();
}

void MyImGui::draw_control_panel() {
	ImGui::Begin("control panel", &show_control_panel_, ImGuiWindowFlags_AlwaysAutoResize);

	if (ImGui::CollapsingHeader("model operators", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::Button("load .obj")) {
			load_obj_file();
		}
	}
	if (isLoadOBJ) {
		if (ImGui::Button("next mesh")) {
			next_mesh();
		}
		ImGui::SameLine();
		if (ImGui::Button("last mesh")) {
			last_mesh();
		}
	}
	if (isParam) {
		if (ImGui::Button("load param")) {
			load_param();
		}
		
	}
	if (isComputeTMesh) {
		if (ImGui::Button("next TMesh")) {
			next_tmesh();
		}
		if (ImGui::Button("last TMesh")) {
			last_tmesh();
		}
	}
	if (isParam && isComputeTMesh) {
		if (ImGui::Button("load result")) {
			load_result();
		}
		ImGui::SameLine();
		if (ImGui::Button("load result quad")) {
			load_result_quad();
		}
		if (ImGui::Button("refine")) {
			double input_epsilon = 0.0001;
			unsigned int min_point_count = 20;
			Global_LSM(input_epsilon, min_point_count);
		}
	}

	//render settings
	if (ImGui::CollapsingHeader("renderer setting", ImGuiTreeNodeFlags_DefaultOpen)) {
		static float bg_color[3] = { 0.2f,0.2f,0.2f };
		if (ImGui::ColorEdit3("background color", bg_color)) {
			set_background_color(easy3d::vec4(bg_color[0], bg_color[1], bg_color[2], 1.0f));
		}
		static bool show_axes = true;
		if (drawable_axes_) {
			show_axes = drawable_axes_->is_visible();
		}
		if (ImGui::Checkbox("show axes", &show_axes)) {
			if (drawable_axes_) {
				drawable_axes_->set_visible(show_axes);
			}
		}

		static bool show_logo = true;
		if (ImGui::Checkbox("show logo", &show_logo)) {
			show_easy3d_logo_ = show_logo;
		}
	}

	ImGui::End();
}

void MyImGui::draw_info_panel() {
	ImGui::Begin("info panel", &show_info_panel_, ImGuiWindowFlags_AlwaysAutoResize);

	//model info
	ImGui::Text("models number:%zu", models().size());

	if (!models().empty()) {
		auto current_model = this->current_model();
		if (current_model) {
			ImGui::Text("current model:%s", current_model->name().c_str());

			if (auto point_cloud = dynamic_cast<easy3d::PointCloud*>(current_model)) {
				ImGui::Text("the number of vertices in pointcloud: %zu", point_cloud->n_vertices());
			}
			else if (auto mesh = dynamic_cast<easy3d::SurfaceMesh*>(current_model)) {
				ImGui::Text("the number of vertices in mesh: %zu", mesh->n_vertices());
				ImGui::Text("the number of edges in mesh: %zu", mesh->n_edges());
				ImGui::Text("the number of faces in mesh: %zu", mesh->n_faces());
			}
		}
	}

	ImGui::Separator();
	ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
	ImGui::Text("window size: %d x  %d", width(), height());
	ImGui::End();
}