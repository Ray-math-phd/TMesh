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

void MyImGui::open_model_file() {
	std::vector<std::string> filters = {
		"Mesh Files (*.ply *.obj)", "*.ply *.obj",
		"All Files (*.*)", "*"
	};

	std::string default_path = easy3d::resource::directory() + "/data";

	std::string file_name = easy3d::dialog::open("please choose a file to open", default_path, filters);

	if (!file_name.empty()) {
		clear_scene();

		std::string file_ext = easy3d::file_system::extension(file_name, true);

		if (add_model(file_name)) {
			std::cout << "successfuly loaded:" << file_name << std::endl;
			fit_screen();
		}
		else {
			std::cerr << "can't load file: " << file_name << std::endl;
			easy3d::dialog::message(
				"error",
				"can't load file: " + file_name,
				easy3d::dialog::Choice::ok,
				easy3d::dialog::Type::error
			);
		}
	}
}

void MyImGui::load_obj_file() {
	std::vector<std::string> filters = {
		"OBJ Files (*.obj)", "*.obj",
		"All Files (*.*)", "*"
	};

	std::string default_path = easy3d::resource::directory() + "/data";

	std::string file_name = easy3d::dialog::open("choose obj file: ", default_path, filters);

	if (!file_name.empty()) {
		clear_scene();

		if (add_model(file_name)) {
			std::cout << "successfullly loaded obj file: " << file_name << std::endl;
			fit_screen();
			auto model = current_model();
			if (model) {
				auto mesh = dynamic_cast<easy3d::SurfaceMesh*>(model);
				if (mesh) {
					// Create a deep copy of the mesh
					auto mesh_ptr = std::make_shared<easy3d::SurfaceMesh>(*mesh);
					meshes.push_back(mesh_ptr);
					isLoadOBJ = true;
					std::cout << "DEBUG: meshes size after loading: " << meshes.size() << std::endl;
					std::cout << "DEBUG: Copied mesh has " << mesh_ptr->n_vertices() << " vertices, " 
							  << mesh_ptr->n_faces() << " faces" << std::endl;
					
					// Set renderer
					set_renderers(mesh_ptr);
				} else {
					std::cerr << "Failed to cast model to SurfaceMesh" << std::endl;
				}
			} else {
				std::cerr << "No current model available" << std::endl;
			}
		}
		else {
			std::cerr << "can't load obj file: " << file_name << std::endl;
			easy3d::dialog::message(
				"error",
				"can't load obj file: " + file_name,
				easy3d::dialog::Choice::ok,
				easy3d::dialog::Type::error
			);
		}
	}
}

void MyImGui::next_mesh() {
	models_.clear();
	std::cout << "DEBUG: next_mesh called - meshes.size(): " << meshes.size() << ", mesh_idx: " << mesh_idx << std::endl;
	if (meshes.size() == 0) {
		std::cout << "GUI ERROR: no meshes." << std::endl;
	}
	else {
		if (mesh_idx == -1) {
			mesh_idx = 0;
			std::cout << "DEBUG: Setting mesh_idx to 0, loading mesh[0]" << std::endl;
			auto mesh_ptr = meshes[mesh_idx];
			if (mesh_ptr) {
				std::cout << "DEBUG: Mesh[0] has " << mesh_ptr->n_vertices() << " vertices, " 
						  << mesh_ptr->n_faces() << " faces" << std::endl;
				if (mesh_ptr->n_vertices() == 0 || mesh_ptr->n_faces() == 0) {
					std::cout << "DEBUG: ERROR - Mesh is empty! This should not happen." << std::endl;
					return;
				}
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
					fit_screen(mesh_ptr.get());
					std::cout << "DEBUG: Successfully loaded mesh[0]" << std::endl;
				} else {
					std::cout << "DEBUG: ERROR - Failed to add model to viewer" << std::endl;
				}
			} else {
				std::cout << "DEBUG: ERROR - mesh_ptr is null" << std::endl;
			}
		}
		else if (mesh_idx >= 0 && mesh_idx < meshes.size() - 1) {
			mesh_idx++;
			std::cout << "DEBUG: Incrementing mesh_idx to " << mesh_idx << ", loading mesh[" << mesh_idx << "]" << std::endl;
			auto mesh_ptr = meshes[mesh_idx];
			if (mesh_ptr) {
				std::cout << "DEBUG: Mesh[" << mesh_idx << "] has " << mesh_ptr->n_vertices() << " vertices, " 
						  << mesh_ptr->n_faces() << " faces" << std::endl;
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
					fit_screen(mesh_ptr.get());
					std::cout << "DEBUG: Successfully loaded mesh[" << mesh_idx << "]" << std::endl;
				}
			}
		}
		else if (mesh_idx == meshes.size() - 1) {
			std::cout << "DEBUG: Already at last mesh[" << mesh_idx << "], keeping current mesh" << std::endl;
			auto mesh_ptr = meshes[mesh_idx];
			if (mesh_ptr) {
				std::cout << "DEBUG: Mesh[" << mesh_idx << "] has " << mesh_ptr->n_vertices() << " vertices, " 
						  << mesh_ptr->n_faces() << " faces" << std::endl;
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
					fit_screen(mesh_ptr.get());
					std::cout << "DEBUG: Reloaded last mesh[" << mesh_idx << "]" << std::endl;
				}
			}
		}
	}
}

void MyImGui::last_mesh() {
	models_.clear();
	std::cout << "DEBUG: last_mesh called - meshes.size(): " << meshes.size() << ", mesh_idx: " << mesh_idx << std::endl;
	if (meshes.size() == 0) {
		std::cout << "GUI ERROR: no meshes." << std::endl;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "GUI ERROR: invalid mesh_idx." << std::endl;
		}
		else if (mesh_idx > 0 && mesh_idx <= meshes.size() - 1) {
			mesh_idx--;
			auto mesh_ptr = meshes[mesh_idx];
			if (mesh_ptr) {
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
					fit_screen(mesh_ptr.get());
				}
			}
		}
		else if (mesh_idx == 0) {
			auto mesh_ptr = meshes[mesh_idx];
			if (mesh_ptr) {
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
					fit_screen(mesh_ptr.get());
				}
			}
		}
	}
}

void MyImGui::load_param() {
	models_.clear();
	std::cout << "DEBUG: load_param called - meshes.size(): " << meshes.size() 
			  << ", mesh_params.size(): " << mesh_params.size() << ", mesh_idx: " << mesh_idx << std::endl;
	
	if (meshes.size() == 0 || mesh_params.size() == 0 || meshes.size() != mesh_params.size()) {
		std::cout << "GUI ERROR: no meshes or mesh_params, or size mismatch." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "GUI ERROR: mesh_idx invalid." << std::endl;
			return;
		}
		else {
			auto mesh_ptr = mesh_params[mesh_idx];
			if (mesh_ptr) {
				std::cout << "DEBUG: Param mesh[" << mesh_idx << "] has " << mesh_ptr->n_vertices() << " vertices, " 
						  << mesh_ptr->n_faces() << " faces" << std::endl;
				if (mesh_ptr->n_vertices() == 0 || mesh_ptr->n_faces() == 0) {
					std::cout << "DEBUG: ERROR - Param mesh is empty! This should not happen." << std::endl;
					return;
				}
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
					fit_screen(mesh_ptr.get());
					std::cout << "DEBUG: Successfully loaded param mesh[" << mesh_idx << "]" << std::endl;
				} else {
					std::cout << "DEBUG: ERROR - Failed to add param model to viewer" << std::endl;
				}
			} else {
				std::cout << "DEBUG: ERROR - param mesh_ptr is null" << std::endl;
			}
		}
	}
}

void MyImGui::next_param() {
	models_.clear();
	if (meshes.size() == 0 && mesh_params.size() == 0 && meshes.size()!= mesh_params.size()) {
		std::cout << "GUI ERROR: no meshes." << std::endl;
	}
	else {
		if (mesh_idx == -1) {
			mesh_idx = 0;
			auto mesh_ptr = mesh_params[mesh_idx];
			if (mesh_ptr) {
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
				}
			}
		}
		else if (mesh_idx >= 0 && mesh_idx < meshes.size() - 1) {
			mesh_idx++;
			auto mesh_ptr = mesh_params[mesh_idx];
			if (mesh_ptr) {
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
				}
			}
		}
		else if (mesh_idx == meshes.size() - 1) {
			auto mesh_ptr = mesh_params[mesh_idx];
			if (mesh_ptr) {
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
				}
			}
		}
	}
}

void MyImGui::last_param() {
	models_.clear();
	if (meshes.size() == 0 || mesh_params.size() == 0 || meshes.size() != mesh_params.size()) {
		std::cout << "GUI ERROR: no meshes." << std::endl;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "GUI ERROR: invalid mesh_idx." << std::endl;
		}
		else if (mesh_idx > 0 && mesh_idx <= meshes.size() - 1) {
			mesh_idx--;
			auto mesh_ptr = mesh_params[mesh_idx];
			if (mesh_ptr) {
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
				}
			}
		}
		else if (mesh_idx == 0) {
			auto mesh_ptr = mesh_params[mesh_idx];
			if (mesh_ptr) {
				if (add_model(mesh_ptr, true)) {
					set_renderers(mesh_ptr);
				}
			}
			
		}
	}
}

void MyImGui::next_tmesh() {
	models_.clear();
	std::cout << "DEBUG: next_tmesh called - meshes.size(): " << meshes.size() 
			  << ", mesh_tsps.size(): " << mesh_tsps.size() 
			  << ", mesh_params.size(): " << mesh_params.size() 
			  << ", mesh_idx: " << mesh_idx 
			  << ", mesh_tsp_idx: " << mesh_tsp_idx << std::endl;
	
	if (meshes.size() == 0 || mesh_tsps.size() == 0 || mesh_params.size() == 0 ||
		meshes.size() != mesh_params.size() || meshes.size() != mesh_tsps.size()) {
		std::cout << "GUI ERROR: no meshes or size mismatch." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "GUI ERROR: invalid mesh_idx." << std::endl;
			return;
		}
		else {
			auto& mesh_tsps_ = mesh_tsps[mesh_idx];
			if (mesh_tsps_.size() == 0) {
				std::cout << "GUI ERROR: no meshes." << std::endl;
				return;
			}
			else {
				if (mesh_tsp_idx == -1) {
					mesh_tsp_idx = 0;
					std::cout << "DEBUG: Setting mesh_tsp_idx to 0, loading tmesh[0]" << std::endl;
					easy3d::SurfaceMesh mesh_tsp;
					auto tsp_ = mesh_tsps[mesh_idx][mesh_tsp_idx].get();
					const tspline::Tspline tsp = *tsp_;
					std::cout << "DEBUG: Converting T-spline to SurfaceMesh..." << std::endl;
					
					// Debug: Output T-spline boundary information
					std::cout << "DEBUG: T-spline info - vertices: " << tsp.number_of_vertices() 
							  << ", faces: " << tsp.number_of_faces() << std::endl;
					
					// Get face bounding box information
					try {
						if (tsp.number_of_faces() > 0) {
							auto face_it = tsp.faces_begin();
							tspline::Point2d bbmin, bbmax;
							tsp.face_bounding_box(face_it, bbmin, bbmax);
							std::cout << "DEBUG: T-spline face bounding box: min=(" << bbmin.x() << ", " << bbmin.y() 
									  << "), max=(" << bbmax.x() << ", " << bbmax.y() << ")" << std::endl;
						}
					} catch (const std::exception& e) {
						std::cout << "DEBUG: Error getting face bounding box: " << e.what() << std::endl;
					} catch (...) {
						std::cout << "DEBUG: Unknown error getting face bounding box" << std::endl;
					}
					
					convert_tsp_2_surfacemesh(tsp, mesh_tsp);
					//debug_tmesh_info(tsp, mesh_tsp);
					
					if (mesh_tsp.n_vertices() == 0 || mesh_tsp.n_faces() == 0) {
						std::cout << "DEBUG: ERROR - T-mesh is empty! This should not happen." << std::endl;
						return;
					}
					
					std::shared_ptr<easy3d::SurfaceMesh> mesh_tsp_ptr = std::make_shared<easy3d::SurfaceMesh>(mesh_tsp);
					if (mesh_tsp_ptr) {
						if (add_model(mesh_tsp_ptr, true)) {
							set_renderers(mesh_tsp_ptr);
							fit_screen(mesh_tsp_ptr.get());
							std::cout << "DEBUG: Successfully loaded tmesh[0]" << std::endl;
						} else {
							std::cout << "DEBUG: ERROR - Failed to add tmesh model to viewer" << std::endl;
						}
					} else {
						std::cout << "DEBUG: ERROR - mesh_tsp_ptr is null" << std::endl;
					}
				}
				else if (mesh_tsp_idx >= 0 && mesh_tsp_idx < mesh_tsps_.size() - 1) {
					mesh_tsp_idx++;
					std::cout << "DEBUG: Incrementing mesh_tsp_idx to " << mesh_tsp_idx << ", loading tmesh[" << mesh_tsp_idx << "]" << std::endl;
					easy3d::SurfaceMesh mesh_tsp;
					auto tsp_ = mesh_tsps[mesh_idx][mesh_tsp_idx].get();
					const tspline::Tspline tsp = *tsp_;
					std::cout << "DEBUG: Converting T-spline to SurfaceMesh..." << std::endl;
					convert_tsp_2_surfacemesh(tsp, mesh_tsp);
					//debug_tmesh_info(tsp, mesh_tsp);
					
					if (mesh_tsp.n_vertices() == 0 || mesh_tsp.n_faces() == 0) {
						std::cout << "DEBUG: ERROR - T-mesh is empty! This should not happen." << std::endl;
						return;
					}
					
					std::shared_ptr<easy3d::SurfaceMesh> mesh_tsp_ptr = std::make_shared<easy3d::SurfaceMesh>(mesh_tsp);
					if (mesh_tsp_ptr) {
						if (add_model(mesh_tsp_ptr, true)) {
							set_renderers(mesh_tsp_ptr);
							fit_screen(mesh_tsp_ptr.get());
							std::cout << "DEBUG: Successfully loaded tmesh[" << mesh_tsp_idx << "]" << std::endl;
						} else {
							std::cout << "DEBUG: ERROR - Failed to add tmesh model to viewer" << std::endl;
						}
					} else {
						std::cout << "DEBUG: ERROR - mesh_tsp_ptr is null" << std::endl;
					}
				}
				else if (mesh_tsp_idx == mesh_tsps_.size() - 1) {
					std::cout << "DEBUG: Already at last tmesh[" << mesh_tsp_idx << "], keeping current tmesh" << std::endl;
					easy3d::SurfaceMesh mesh_tsp;
					auto tsp_ = mesh_tsps[mesh_idx][mesh_tsp_idx].get();
					const tspline::Tspline tsp = *tsp_;
					std::cout << "DEBUG: Converting T-spline to SurfaceMesh..." << std::endl;
					convert_tsp_2_surfacemesh(tsp, mesh_tsp);
					//debug_tmesh_info(tsp, mesh_tsp);
					
					if (mesh_tsp.n_vertices() == 0 || mesh_tsp.n_faces() == 0) {
						std::cout << "DEBUG: ERROR - T-mesh is empty! This should not happen." << std::endl;
						return;
					}
					
					std::shared_ptr<easy3d::SurfaceMesh> mesh_tsp_ptr = std::make_shared<easy3d::SurfaceMesh>(mesh_tsp);
					if (mesh_tsp_ptr) {
						if (add_model(mesh_tsp_ptr, true)) {
							set_renderers(mesh_tsp_ptr);
							fit_screen(mesh_tsp_ptr.get());
							std::cout << "DEBUG: Reloaded last tmesh[" << mesh_tsp_idx << "]" << std::endl;
						} else {
							std::cout << "DEBUG: ERROR - Failed to add tmesh model to viewer" << std::endl;
						}
					} else {
						std::cout << "DEBUG: ERROR - mesh_tsp_ptr is null" << std::endl;
					}
				}
			}
		}
	}

}

void MyImGui::last_tmesh() {
	models_.clear();
	std::cout << "DEBUG: last_tmesh called - meshes.size(): " << meshes.size() 
			  << ", mesh_tsps.size(): " << mesh_tsps.size() 
			  << ", mesh_params.size(): " << mesh_params.size() 
			  << ", mesh_idx: " << mesh_idx 
			  << ", mesh_tsp_idx: " << mesh_tsp_idx << std::endl;
	
	if (meshes.size() == 0 || mesh_tsps.size() == 0 || mesh_params.size() == 0 ||
		meshes.size() != mesh_params.size() || meshes.size() != mesh_tsps.size()) {
		std::cout << "GUI ERROR: no meshes or size mismatch." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "GUI ERROR: invalid mesh_idx." << std::endl;
			return;
		}
		else {
			auto& mesh_tsps_ = mesh_tsps[mesh_idx];
			if (mesh_tsps_.size() == 0) {
				std::cout << "GUI ERROR: no meshes." << std::endl;
				return;
			}
			else {
				if (mesh_tsp_idx == -1) {
					std::cout << "GUI ERROR: no meshes." << std::endl;
					return;
				}
				else if (mesh_tsp_idx > 0 && mesh_tsp_idx <= mesh_tsps_.size() - 1) {
					mesh_tsp_idx--;
					std::cout << "DEBUG: Decrementing mesh_tsp_idx to " << mesh_tsp_idx << ", loading tmesh[" << mesh_tsp_idx << "]" << std::endl;
					easy3d::SurfaceMesh mesh_tsp;
					auto tsp_ = mesh_tsps[mesh_idx][mesh_tsp_idx].get();
					const tspline::Tspline tsp = *tsp_;
					std::cout << "DEBUG: Converting T-spline to SurfaceMesh..." << std::endl;
					convert_tsp_2_surfacemesh(tsp, mesh_tsp);
					//debug_tmesh_info(tsp, mesh_tsp);
					
					if (mesh_tsp.n_vertices() == 0 || mesh_tsp.n_faces() == 0) {
						std::cout << "DEBUG: ERROR - T-mesh is empty! This should not happen." << std::endl;
						return;
					}
					
					std::shared_ptr<easy3d::SurfaceMesh> mesh_tsp_ptr = std::make_shared<easy3d::SurfaceMesh>(mesh_tsp);
					if (mesh_tsp_ptr) {
						if (add_model(mesh_tsp_ptr, true)) {
							set_renderers(mesh_tsp_ptr);
							fit_screen(mesh_tsp_ptr.get());
							std::cout << "DEBUG: Successfully loaded tmesh[" << mesh_tsp_idx << "]" << std::endl;
						} else {
							std::cout << "DEBUG: ERROR - Failed to add tmesh model to viewer" << std::endl;
						}
					} else {
						std::cout << "DEBUG: ERROR - mesh_tsp_ptr is null" << std::endl;
					}
				}
				else if (mesh_tsp_idx == 0) {
					std::cout << "DEBUG: Already at first tmesh[0], keeping current tmesh" << std::endl;
					easy3d::SurfaceMesh mesh_tsp;
					auto tsp_ = mesh_tsps[mesh_idx][mesh_tsp_idx].get();
					const tspline::Tspline tsp = *tsp_;
					std::cout << "DEBUG: Converting T-spline to SurfaceMesh..." << std::endl;
					convert_tsp_2_surfacemesh(tsp, mesh_tsp);
					//debug_tmesh_info(tsp, mesh_tsp);
					
					if (mesh_tsp.n_vertices() == 0 || mesh_tsp.n_faces() == 0) {
						std::cout << "DEBUG: ERROR - T-mesh is empty! This should not happen." << std::endl;
						return;
					}
					
					std::shared_ptr<easy3d::SurfaceMesh> mesh_tsp_ptr = std::make_shared<easy3d::SurfaceMesh>(mesh_tsp);
					if (mesh_tsp_ptr) {
						if (add_model(mesh_tsp_ptr, true)) {
							set_renderers(mesh_tsp_ptr);
							fit_screen(mesh_tsp_ptr.get());
							std::cout << "DEBUG: Reloaded first tmesh[0]" << std::endl;
						} else {
							std::cout << "DEBUG: ERROR - Failed to add tmesh model to viewer" << std::endl;
						}
					} else {
						std::cout << "DEBUG: ERROR - mesh_tsp_ptr is null" << std::endl;
					}
				}
			}
		}
	}
}

void MyImGui::load_result() {
	models_.clear();
	if (meshes.size() == 0 || mesh_tsps.size() == 0 || mesh_params.size() == 0 ||
		meshes.size() != mesh_params.size() || meshes.size() != mesh_tsps.size() ||
		mesh_results.size() != meshes.size()) {
		std::cout << "GUI ERROR: no meshes." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "GUI ERROR: no meshes." << std::endl;
			return;
		}
		else {
			auto& mesh_results_ = mesh_results[mesh_idx];
			int mesh_result_idx = mesh_tsp_idx;
			if (mesh_results_.size() == 0 || mesh_results_.size() != mesh_tsps[mesh_idx].size()) {
				std::cout << "GUI ERROR: no meshes." << std::endl;
				return;
			}
			else {
				if (mesh_result_idx == -1) {
					std::cout << "GUI ERROR: invalid mesh_tsp_idx." << std::endl;
				}
				else {
					auto result_ptr = mesh_results[mesh_idx][mesh_result_idx];
					if (result_ptr) {
						if (add_model(result_ptr, true)) {
							set_renderers(result_ptr);
							fit_screen(result_ptr.get());
						}
					}
				}
			}
		}
	}
}

void MyImGui::load_result_quad() {
	models_.clear();
	if (meshes.size() == 0 || mesh_tsps.size() == 0 || mesh_params.size() == 0 ||
		meshes.size() != mesh_params.size() || meshes.size() != mesh_tsps.size() ||
		mesh_results_quad.size() != meshes.size()) {
		std::cout << "GUI ERROR: no meshes." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "GUI ERROR: no meshes." << std::endl;
			return;
		}
		else {
			auto& mesh_results_quad_ = mesh_results_quad[mesh_idx];
			int mesh_result_quad_idx = mesh_tsp_idx;
			std::cout << "[DEBUG] load_result_quad - mesh_idx: " << mesh_idx << std::endl;
			std::cout << "[DEBUG] load_result_quad - mesh_results_quad[" << mesh_idx << "].size(): " << mesh_results_quad[mesh_idx].size() << std::endl;
			std::cout << "[DEBUG] load_result_quad - mesh_tsps[" << mesh_idx << "].size(): " << mesh_tsps[mesh_idx].size() << std::endl;
			std::cout << "[DEBUG] load_result_quad - mesh_tsp_idx: " << mesh_tsp_idx << std::endl;
			if (mesh_results_quad[mesh_idx].size() == 0 || mesh_results_quad[mesh_idx].size() != mesh_tsps[mesh_idx].size()) {
				std::cout << "GUI ERROR: no meshes." << std::endl;
				return;
			}
			else {
				if (mesh_result_quad_idx == -1) {
					std::cout << "GUI ERROR: invalid mesh_tsp_idx." << std::endl;
				}
				else {
					auto result_quad_ptr = mesh_results_quad[mesh_idx][mesh_result_quad_idx];
					if (result_quad_ptr) {
						if (add_model(result_quad_ptr, true)) {
							set_renderers(result_quad_ptr);
							fit_screen(result_quad_ptr.get());
						}
					}
				}
			}
		}
	}
}

void MyImGui::save_model() {
	const std::vector<std::string> filters = {
		"OBJ files (*.obj)","*.obj",
		"ALL files (*.*)","*"
	};
	std::string filename = easy3d::dialog::save("save mesh", "mesh.obj", filters, true);
	if (filename.empty()) {
		std::cout << "canceal the save operation." << std::endl;
	}

	save_model(filename);
}

void MyImGui::save_model(const std::string& filename) {
	auto model = current_model();
	if (auto mesh = dynamic_cast<easy3d::SurfaceMesh*>(model)) {
		bool success = easy3d::SurfaceMeshIO::save(filename, mesh);
		if (success) {
			std::cout << "mesh has been saved in " << filename << std::endl;
		}
		else {
			std::cerr << "save fault." << std::endl;
		}
	}
}

void MyImGui::clear_all() {
	clear_scene();

	models_.clear();

	isLoadOBJ = false;
	isParam = false;
	isComputeTMesh = false;

	mesh_idx = -1;
	mesh_tsp_idx = -1;
	
	meshes.clear();
	mesh_params.clear();
	mesh_tsps.clear();
	mesh_results.clear();
	mesh_results_quad.clear();
}


//==================================================== renderer setting =====================================
void MyImGui::set_renderers(std::shared_ptr<easy3d::SurfaceMesh>& model) {
	std::cout << "DEBUG: set_renderers called for model: " << model->name() << std::endl;
	auto renderer = model->renderer();
	if (renderer) {
		std::cout << "DEBUG: Renderer found, setting visible to true" << std::endl;
		renderer->set_visible(true);

		// Check if it's a SurfaceMesh
		if (auto mesh = std::dynamic_pointer_cast<easy3d::SurfaceMesh>(model)) {
			std::cout << "DEBUG: Model is SurfaceMesh, vertices: " << mesh->n_vertices() 
					  << ", faces: " << mesh->n_faces() << ", edges: " << mesh->n_edges() << std::endl;
			// Set edge rendering
			static bool show_edges = true;
			static float line_width = 1.0f;
			auto edges_drawable = model->renderer()->get_lines_drawable("edges");
			if (!edges_drawable) {
				std::cout << "DEBUG: Creating edge drawable" << std::endl;
				// If not exists, create edge drawable
				edges_drawable = model->renderer()->add_lines_drawable("edges");
			}
			if (edges_drawable) {
				std::cout << "DEBUG: Setting edge drawable visible: " << show_edges << ", line_width: " << line_width << std::endl;
				edges_drawable->set_visible(show_edges);
				edges_drawable->set_line_width(line_width);
			} else {
				std::cout << "DEBUG: ERROR - Failed to get or create edge drawable" << std::endl;
			}

			// Set face rendering (most important for mesh display)
			auto faces_drawable = model->renderer()->get_triangles_drawable("faces");
			if (!faces_drawable) {
				std::cout << "DEBUG: Creating face drawable" << std::endl;
				faces_drawable = model->renderer()->add_triangles_drawable("faces");
			}
			if (faces_drawable) {
				std::cout << "DEBUG: Setting face drawable visible and properties" << std::endl;
				faces_drawable->set_visible(true);
				faces_drawable->set_uniform_coloring(easy3d::vec4(0.7f, 0.7f, 0.7f, 1.0f)); // Light gray color
				faces_drawable->set_smooth_shading(true);
			} else {
				std::cout << "DEBUG: ERROR - Failed to get or create face drawable" << std::endl;
			}

			// Set vertex rendering
			static bool show_vertices = false;  // Default: don't show vertices
			static float vertex_size = 1.0f;
			auto vertices_drawable = model->renderer()->get_points_drawable("vertices");
			if (!vertices_drawable) {
				std::cout << "DEBUG: Creating vertex drawable" << std::endl;
				// If not exists, create vertex drawable
				vertices_drawable = model->renderer()->add_points_drawable("vertices");
			}
			if (vertices_drawable) {
				std::cout << "DEBUG: Setting vertex drawable visible: " << show_vertices << ", size: " << vertex_size << std::endl;
				vertices_drawable->set_visible(show_vertices);
				vertices_drawable->set_point_size(vertex_size);
			} else {
				std::cout << "DEBUG: ERROR - Failed to get or create vertex drawable" << std::endl;
			}
		}
		// Check if it's a PointCloud
		else if (auto cloud = std::dynamic_pointer_cast<easy3d::PointCloud>(model)) {
			// Set point cloud rendering
			static float point_size = 1.0f;
			auto points_drawable = model->renderer()->get_points_drawable("vertices");
			if (!points_drawable) {
				// If not exists, create point cloud drawable
				points_drawable = model->renderer()->add_points_drawable("vertices");
			}
			if (points_drawable) {
				points_drawable->set_visible(true);
				points_drawable->set_point_size(point_size);
			}
		}
	}
}

//================================================== precompute functions =============================================
//================================================== designed for control panel 1 =====================================
void MyImGui::load_txt_file(const std::string& filename) {
	std::vector<std::string> filters = {
		"Text Files (*.txt)","*.txt",
		"All Files (*.*)","*"
	};
	std::string default_path = easy3d::resource::directory() + "/data";
	std::string file_name = easy3d::dialog::open("choose auxiliary file: ", default_path, filters);
	if (filename.empty()) {//safe check.
		std::cout << "error in load_txt_file: filename is empty." << std::endl;
		return;
	}
	std::string file_ext = easy3d::file_system::extension(file_name, true);
	if (file_ext != "txt") {
		std::cout << "error in load_txt_file: file_ext is invalid." << std::endl;
		return;
	}

	std::cout << "[load_txt_file]: reading txt file: " << filename << std::endl;
	auto rootmesh_ = rootmesh.get();//the deep copy of the class var rootmesh.

	std::vector<std::string> lines; lines.clear();

	std::ifstream file(filename);
	if (!file.is_open()) {//safe check.
		std::cout << "[load_txt_file]: can't open the txt file." << std::endl;
		return;
	}

	std::string line;
	while (std::getline(file, line)) {
		if (!line.empty() && line.back() == '\r') { line.pop_back(); }
		lines.push_back(line);
	}
	file.close();//safe.

	rootmesh_patches.clear();

	std::vector<int> lines_int;
	lines_int.reserve(lines.size());
	for (const auto& line_ : lines) {
		int value;
		auto result = std::from_chars(line_.data(), line_.data() + line_.size(), value);
		if (result.ec == std::errc()) { lines_int.push_back(value); }
		else {
			std::cout << "[load_txt_file]: transform error." << std::endl;
			return;
		}
	}

	if (lines_int.size() != rootmesh_->n_faces()) {//safe.
		std::cout << "[load_txt_file]:occur error because open wrong configure (txt)file: " << filename << std::endl;
		return;
	}

	//0. the unique_patches store the number of the patches.
	std::set<int> unique_patches; unique_patches.clear();
	for (int patch_index : lines_int) { unique_patches.insert(patch_index); }

	//1. collect all faces and vertices of per patch which will be stored in the per_patch_faces and per_patch_vertices.
	std::map<int, std::vector<int>> per_patch_faces;
	std::map<int, std::set<int>> per_patch_vertices;
	for (size_t face_idx = 0; face_idx < lines_int.size(); face_idx++) {
		int patch_idx = lines_int[face_idx];
		per_patch_faces[patch_idx].push_back(face_idx);
		auto face = get_mesh_face_by_idx(rootmesh_, face_idx);
		if (face.is_valid()) {//safe.
			for (auto v : rootmesh_->vertices(face)) {
				per_patch_vertices[patch_idx].insert(v.idx());
			}
		}
	}

	//2.
	int curr_meshes_size = meshes.size();
	int start_patch_index_in_meshes = curr_meshes_size;
	for (int patch_idx : unique_patches) {
		auto per_patch_mesh = new easy3d::SurfaceMesh();
		std::map<int, int> vertex_map;//important map which store the map between the new index in per_patch_mesh and the rootmesh.
		int new_vertex_idx(0);//the new index in the per_patch_mesh.
		//2.1. construct the vertices for per patch.
		for (int old_vertex_idx : per_patch_vertices[patch_idx]) {
			auto old_vertex = get_mesh_vertex_by_idx(rootmesh_, old_vertex_idx);
			if (old_vertex.is_valid()) {
				auto new_vertex = per_patch_mesh->add_vertex(rootmesh->position(old_vertex));
				vertex_map[old_vertex_idx] = new_vertex_idx;
				new_vertex_idx++;
			}
		}
		//2.2 construct the faces for per patch.
		for (int face_idx : per_patch_faces[patch_idx]) {
			auto face = get_mesh_face_by_idx(rootmesh_, face_idx);
			if (face.is_valid()) {
				std::vector<easy3d::SurfaceMesh::Vertex> new_face_vertices;
				for (auto v : rootmesh_->vertices(face)) {
					int old_idx = v.idx();
					auto map_it = vertex_map.find(old_idx);
					if (map_it != vertex_map.end()) {
						auto new_vertex = get_mesh_vertex_by_idx(per_patch_mesh, map_it->second);
						new_face_vertices.push_back(new_vertex);
					}
				}
				if (new_face_vertices.size() == 3) {//safe.
					auto new_face = per_patch_mesh->add_face(new_face_vertices);
					if (!new_face.is_valid()) {
						std::cout << "[load_txt_file]: Failed to add new face to patch: " << patch_idx << std::endl;
					}
				}
			}
		}
		//2.3 add the per_patch_mesh into the meshes.
		meshes.push_back(std::shared_ptr<easy3d::SurfaceMesh>(per_patch_mesh));
		std::cout << "Created patch " << patch_idx << " with "
			<< per_patch_mesh->n_vertices() << " vertices and "
			<< per_patch_mesh->n_faces() << " faces" << std::endl;
	}
	std::cout << "Successfully search total " << unique_patches.size() << " patches, Successfully add total" << meshes.size() - start_patch_index_in_meshes << " patches" << std::endl;
	std::cout << "[load_txt_file]: finish reading txt file. " << filename << std::endl;
}





//================================================== convert funtcion =================================================
void MyImGui::convert_vec3_2_dvec3(easy3d::SurfaceMesh* mesh, std::vector<easy3d::dvec3>& param_dvec3) {
	param_dvec3.clear();
	for (auto vit : mesh->vertices()) {
		auto pos = mesh->position(vit);
		easy3d::dvec3 pos_d = static_cast<easy3d::dvec3>(pos);
		param_dvec3.push_back(pos_d);
	}
}

void MyImGui::convert_tsp_2_surfacemesh(const tspline::Tspline& tsp, easy3d::SurfaceMesh& mesh) {
	mesh.clear();

	std::unordered_map<const void*, easy3d::SurfaceMesh::Vertex> vertex_map;

	for (auto fit = tsp.faces_begin(); fit != tsp.faces_end(); fit++) {
		if (!fit->is_unbounded() && fit->has_outer_ccb()) {
			auto first = fit->outer_ccb();
			auto curr = first;
			std::vector<easy3d::SurfaceMesh::Vertex> face_vertices;

			do {
				auto vit = curr->source();
				const tspline::TVertex& tvertex = vit->data();

				Eigen::Vector3d cp;
				tvertex.GetCP(cp);

				auto it = vertex_map.find(&(*vit));
				easy3d::SurfaceMesh::Vertex v;

				if (it == vertex_map.end()) {
					// Normalize parameter coordinates to [0,1] x [0,1] range
					//double norm_x = tvertex.param.x() / 3.0;  // Assuming max x is 3
					//double norm_y = tvertex.param.y() / 3.0;  // Assuming max y is 3
					v = mesh.add_vertex(easy3d::vec3(tvertex.param.x(), tvertex.param.y(), 0.0));
					vertex_map[&(*vit)] = v;
				}
				else {
					v = it->second;
				}

				face_vertices.push_back(v);
			} while (++curr != first);

			if (face_vertices.size() >= 3) {
				mesh.add_face(face_vertices);
			}
		}
	}
}

//================================================== Parameterization funtcions =================================================

void MyImGui::Parameterization_uniform_weight() {
	if (meshes.size() == 0) {
		std::cout << "no meshes." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "invalid mesh_idx." << std::endl;
			return;
		}
		else {
			int Type = 1;
			auto new_mesh = std::make_shared<easy3d::SurfaceMesh>();
			easy3d::SurfaceMesh* mesh = meshes[mesh_idx].get();
			myparam::MyParam my_param(mesh);
			my_param.Parameterization(Type);
			my_param.getParam(*new_mesh);
			mesh_params.push_back(new_mesh);
			isParam = true;
		}
	}
}
void MyImGui::Parameterization_chord_weight() {
	if (meshes.size() == 0) {
		std::cout << "no meshes." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "invalid mesh_idx." << std::endl;
			return;
		}
		else {
			int Type = 2;
			auto new_mesh = std::make_shared<easy3d::SurfaceMesh>();
			easy3d::SurfaceMesh* mesh = meshes[mesh_idx].get();
			myparam::MyParam my_param(mesh);
			my_param.Parameterization(Type);
			my_param.getParam(*new_mesh);
			mesh_params.push_back(new_mesh);
			isParam = true;
		}
	}
}

//==================================================== debug T-mesh information ===============================
void MyImGui::debug_tmesh_info(const tspline::Tspline& tsp, const easy3d::SurfaceMesh& mesh_tsp) {
	std::cout << "DEBUG: T-spline info - vertices: " << tsp.number_of_vertices() 
			  << ", faces: " << tsp.number_of_faces() << std::endl;
	
	// Get face bounding box information for the first face
	try {
		if (tsp.number_of_faces() > 0) {
			auto face_it = tsp.faces_begin();
			tspline::Point2d bbmin, bbmax;
			tsp.face_bounding_box(face_it, bbmin, bbmax);
			std::cout << "DEBUG: T-spline face bounding box: min=(" << bbmin.x() << ", " << bbmin.y() 
					  << "), max=(" << bbmax.x() << ", " << bbmax.y() << ")" << std::endl;
		}
	} catch (const std::exception& e) {
		std::cout << "DEBUG: Error getting face bounding box: " << e.what() << std::endl;
	} catch (...) {
		std::cout << "DEBUG: Unknown error getting face bounding box" << std::endl;
	}
	
	std::cout << "DEBUG: Converted tmesh has " << mesh_tsp.n_vertices() << " vertices, " 
			  << mesh_tsp.n_faces() << " faces" << std::endl;
	
	// Debug: Output T-spline parameter coordinates
	std::cout << "DEBUG: T-spline parameter coordinates:" << std::endl;
	int vertex_idx = 0;
	for (auto vit = tsp.vertices_begin(); vit != tsp.vertices_end(); ++vit, ++vertex_idx) {
		const auto& point = vit->point();
		std::cout << "  Vertex " << vertex_idx << ": (" 
				  << point.x() << ", " << point.y() << ")" << std::endl;
	}
	
	// Debug: Output converted mesh vertex coordinates
	std::cout << "DEBUG: Converted T-mesh vertex coordinates:" << std::endl;
	vertex_idx = 0;
	for (auto vit = mesh_tsp.vertices_begin(); vit != mesh_tsp.vertices_end(); ++vit, ++vertex_idx) {
		const auto& pos = mesh_tsp.position(*vit);
		std::cout << "  Vertex " << vertex_idx << ": (" 
				  << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
	}
	
	// Debug: Output face information
	std::cout << "DEBUG: T-mesh face information:" << std::endl;
	int face_idx = 0;
	for (auto fit = mesh_tsp.faces_begin(); fit != mesh_tsp.faces_end(); ++fit, ++face_idx) {
		std::cout << "  Face " << face_idx << ": ";
		// Use the correct API for iterating vertices around a face
		auto cir = mesh_tsp.vertices(*fit);
		auto end = cir;
		do {
			// Find vertex index by counting from beginning
			int v_idx = 0;
			for (auto vit = mesh_tsp.vertices_begin(); vit != mesh_tsp.vertices_end(); ++vit, ++v_idx) {
				if (*vit == *cir) {
					std::cout << v_idx << " ";
					break;
				}
			}
			++cir;
		} while (cir != end);
		std::cout << std::endl;
	}
	
	// Debug: Output edge information
	std::cout << "DEBUG: T-mesh edge information:" << std::endl;
	int edge_idx = 0;
	for (auto eit = mesh_tsp.edges_begin(); eit != mesh_tsp.edges_end(); ++eit, ++edge_idx) {
		auto v1 = mesh_tsp.vertex(*eit, 0);
		auto v2 = mesh_tsp.vertex(*eit, 1);
		
		// Find vertex indices
		int v1_idx = 0, v2_idx = 0;
		for (auto vit = mesh_tsp.vertices_begin(); vit != mesh_tsp.vertices_end(); ++vit, ++v1_idx) {
			if (*vit == v1) break;
		}
		for (auto vit = mesh_tsp.vertices_begin(); vit != mesh_tsp.vertices_end(); ++vit, ++v2_idx) {
			if (*vit == v2) break;
		}
		
		std::cout << "  Edge " << edge_idx << ": " << v1_idx << " - " << v2_idx << std::endl;
	}
}

//==================================================== compute control points ===============================
void MyImGui::compute_control_points(tspline::Tspline& tsp, std::vector<easy3d::dvec3>& mesh, std::vector<easy3d::dvec3>& mesh_param) {
	std::cout << "DEBUG: compute_control_points called" << std::endl;
	int row = mesh.size();
	int col = tsp.number_of_vertices();
	std::cout << "DEBUG: Matrix dimensions - row: " << row << ", col: " << col << std::endl;
	
	if (row == 0 || col == 0) {
		std::cout << "ERROR: Invalid matrix dimensions - row: " << row << ", col: " << col << std::endl;
		return;
	}
	
	Eigen::MatrixXd LeftTermxyz_P(row, 3);
	Eigen::SparseMatrix<double> CoeffMatrix_W(row, col);
	std::vector<Eigen::Triplet<double>> triplets;

	//1.init the CoeffMatrix_W
	for (int i = 0; i < row; i++) {
		LeftTermxyz_P(i, 0) = mesh[i].x;
		LeftTermxyz_P(i, 1) = mesh[i].y;
		LeftTermxyz_P(i, 2) = mesh[i].z;

		for (tspline::Tspline::Vertex_const_iterator vit = tsp.vertices_begin(); vit != tsp.vertices_end(); vit++) {
			double s = mesh_param[i].x;
			double t = mesh_param[i].y;

			double b(0.0);
			const tspline::TVertex& vext = vit->data();

			int cp_id = vext.id;

			// Add control point ID range validation
			if (cp_id < 0 || cp_id >= col) {
				std::cout << "Warning: Invalid control point ID: " << cp_id
					<< " (expected range: 0 to " << (col - 1) << "). Skipping." << std::endl;
				continue;
			}

			int col_Matrix_W = cp_id;
			tsp.evaluate_basis(s, t, vext, b);
			if (b != 0.0) {
				triplets.push_back(Eigen::Triplet<double>(i, col_Matrix_W, b));
			}
		}
	}


	CoeffMatrix_W.setFromTriplets(triplets.begin(), triplets.end());

	// Add system checks
	if (CoeffMatrix_W.rows() < CoeffMatrix_W.cols()) {
		std::cout << "Error: Underdetermined system. Rows: " << CoeffMatrix_W.rows()
			<< ", Cols: " << CoeffMatrix_W.cols() << std::endl;
		return;
	}

	if (triplets.empty()) {
		std::cout << "Error: No valid triplets found in coefficient matrix" << std::endl;
		return;
	}

	//2. solve the matrix equation.
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
	qr.compute(CoeffMatrix_W);

	if (qr.info() != Eigen::Success) {
		std::cout << "QR decomposition failed." << std::endl;
		return;
	}

	Eigen::MatrixXd RightTermxyz_C(col, 3);
	for (int i = 0; i < 3; i++) {
		RightTermxyz_C.col(i) = qr.solve(LeftTermxyz_P.col(i));

		if (qr.info() != Eigen::Success) {
			std::cout << "Solving failed for dimension " << i << std::endl;
			return;
		}
	}

	//3.
	//use result(RightTermxyz_C) to init cp_s in tsp.
	for (tspline::Tspline::Vertex_iterator v = tsp.vertices_begin(); v != tsp.vertices_end(); v++) {
		tspline::TVertex& vext = v->data();
		int cp_id = vext.id;

		if (cp_id < 0 || cp_id >= col) {
			std::cout << "Warning: Invalid control point ID during result assignment: " << cp_id
				<< " (expected range: 0 to " << (col - 1) << "). Skipping." << std::endl;
			continue;
		}

		auto vec3 = Eigen::Vector3d(RightTermxyz_C(cp_id, 0), RightTermxyz_C(cp_id, 1), RightTermxyz_C(cp_id, 2));
		//std::cout << "control point: (" << RightTermxyz_C(cp_id, 0) << "," << RightTermxyz_C(cp_id, 1) << "," << RightTermxyz_C(cp_id, 2) << ")." << std::endl;
		vext.SetCP(vec3);
	}

}

void MyImGui::compute_orignal_tmesh(const int segX, const int segY) {
	std::cout << "DEBUG: compute_orignal_tmesh called with segX=" << segX << ", segY=" << segY << std::endl;
	std::cout << "DEBUG: Current state - meshes.size(): " << meshes.size() 
			  << ", mesh_params.size(): " << mesh_params.size() 
			  << ", mesh_idx: " << mesh_idx << std::endl;
	
	mesh_tsps.clear();
	if (meshes.size() == 0 ||
		mesh_params.size() == 0 ||
		meshes.size() != mesh_params.size()) {
		std::cout << "ERROR: no meshes or mesh_params, or size mismatch." << std::endl;
		return;
	}
	else {
		if (mesh_idx == -1) {
			std::cout << "ERROR: invalid mesh_idx." << std::endl;
			return;
		}
		else {
			std::cout << "DEBUG: Using mesh[" << mesh_idx << "] for computation" << std::endl;
			auto& mesh = meshes[mesh_idx];
			auto& mesh_param = mesh_params[mesh_idx];
			
			std::cout << "DEBUG: Mesh data - vertices: " << mesh->n_vertices() 
					  << ", faces: " << mesh->n_faces() << std::endl;
			std::cout << "DEBUG: Param mesh data - vertices: " << mesh_param->n_vertices() 
					  << ", faces: " << mesh_param->n_faces() << std::endl;
			
			std::vector<easy3d::dvec3> mesh_, mesh_param_;
			std::cout << "DEBUG: Converting mesh data to dvec3 format..." << std::endl;
			convert_vec3_2_dvec3(mesh.get(), mesh_);
			convert_vec3_2_dvec3(mesh_param.get(), mesh_param_);
			std::cout << "DEBUG: Converted mesh_ size: " << mesh_.size() 
					  << ", mesh_param_ size: " << mesh_param_.size() << std::endl;
			
			std::cout << "DEBUG: Creating T-spline..." << std::endl;
			auto tsp = std::make_shared<tspline::Tspline>();
			tspline::TsplineCreator tsp_creator;
			std::cout << "DEBUG: Creating clamped plane XY with segX=" << segX << ", segY=" << segY << std::endl;
			
			// Check for invalid parameters that would cause division by zero
			unsigned actualSegX = segX;
			unsigned actualSegY = segY;
			if (segX < 3 || segY < 3) {
				std::cout << "ERROR: segX and segY must be >= 3 for CreateClampedPlaneXY (current: segX=" << segX << ", segY=" << segY << ")" << std::endl;
				std::cout << "DEBUG: Using minimum values segX=3, segY=3 instead" << std::endl;
				actualSegX = 3;
				actualSegY = 3;
			}
			
			try {
				std::cout << "DEBUG: About to call CreateClampedPlaneXY..." << std::endl;
				//tsp_creator.CreateClampedPlaneXY(*tsp, 0.0, 0.0, 0.0, 1.0, 1.0, actualSegX, actualSegY);
				tsp_creator.CreatePlaneXY(*tsp, 0.0, 0.0, 0.0, 1.0, 1.0, actualSegX, actualSegY);
				std::cout << "DEBUG: CreateClampedPlaneXY completed successfully" << std::endl;
				
				std::cout << "DEBUG: Updating vertex IDs..." << std::endl;
				tsp->update_vertex_ids();
				std::cout << "DEBUG: Vertex IDs updated successfully" << std::endl;
				
				std::cout << "DEBUG: T-spline created, vertices: " << tsp->number_of_vertices() << std::endl;
			} catch (const std::exception& e) {
				std::cout << "ERROR: Exception in T-spline creation: " << e.what() << std::endl;
				return;
			} catch (...) {
				std::cout << "ERROR: Unknown exception in T-spline creation" << std::endl;
				return;
			}
			
			std::cout << "DEBUG: Computing control points..." << std::endl;
			try {
				compute_control_points(*tsp, mesh_, mesh_param_);
				std::cout << "DEBUG: Control points computed successfully" << std::endl;
			} catch (const std::exception& e) {
				std::cout << "ERROR: Exception in control points computation: " << e.what() << std::endl;
				return;
			} catch (...) {
				std::cout << "ERROR: Unknown exception in control points computation" << std::endl;
				return;
			}

			std::vector<std::shared_ptr<tspline::Tspline>> tsps;
			tsps.push_back(tsp);
			
			std::cout << "DEBUG: Adding T-spline to mesh_tsps..." << std::endl;
			if (mesh_tsps.size() == 0) {
				mesh_tsps.push_back(tsps);
				std::cout << "DEBUG: Added new T-spline group to mesh_tsps" << std::endl;
			}

			std::cout << "DEBUG: Computing result mesh..." << std::endl;
			try {
				compute_result_mesh(*(tsp.get()), mesh_param.get());
				unsigned segX = 25, segY = 25;
				compute_result_quad_mesh(*(tsp.get()), segX, segY);
				std::cout << "DEBUG: Result mesh computed successfully" << std::endl;
			} catch (const std::exception& e) {
				std::cout << "ERROR: Exception in result mesh computation: " << e.what() << std::endl;
				return;
			} catch (...) {
				std::cout << "ERROR: Unknown exception in result mesh computation" << std::endl;
				return;
			}
			
			isComputeTMesh = true;
			std::cout << "DEBUG: compute_orignal_tmesh completed successfully" << std::endl;
		}
	}
}

void MyImGui::compute_result_mesh(tspline::Tspline& tsp, easy3d::SurfaceMesh* mesh_param) {
	easy3d::SurfaceMesh new_mesh;
	new_mesh.clear();
	for (auto fit : mesh_param->faces()) {
		std::vector<easy3d::SurfaceMesh::Vertex> face_vertices;
		face_vertices.clear();
		std::map<easy3d::SurfaceMesh::Vertex, easy3d::SurfaceMesh::Vertex> global_vertex_map;
		global_vertex_map.clear();
		if (fit.is_valid()) {
			for (auto he : mesh_param->halfedges(fit)) {
				if (he.is_valid()) {
					auto v = mesh_param->target(he);
					auto v_it = global_vertex_map.find(v);
					if (v_it == global_vertex_map.end()) {
						easy3d::dvec3 param_pos = static_cast<easy3d::dvec3>(mesh_param->position(v));
						Eigen::Vector3d result = tsp.evaluate(param_pos.x, param_pos.y);
						float x = static_cast<float>(result.x());
						float y = static_cast<float>(result.y());
						float z = static_cast<float>(result.z());
						easy3d::SurfaceMesh::Vertex new_v = new_mesh.add_vertex(easy3d::vec3(x, y, z));
						global_vertex_map[v] = new_v;
						face_vertices.push_back(new_v);
					}
					else {
						face_vertices.push_back(v_it->second);
					}
				}
			}
		}
		if (face_vertices.size() >= 3) {
			new_mesh.add_face(face_vertices);
		}
	}

	std::shared_ptr<easy3d::SurfaceMesh> new_mesh_ptr = std::make_shared<easy3d::SurfaceMesh>(new_mesh);
	if (mesh_results.size() == 0) {
		std::vector<std::shared_ptr<easy3d::SurfaceMesh>> mesh_ptrs;
		mesh_ptrs.push_back(new_mesh_ptr);
		mesh_results.push_back(mesh_ptrs);
	}
	else {
		mesh_results[mesh_idx].push_back(new_mesh_ptr);
	}
	
}

void MyImGui::compute_result_quad_mesh(tspline::Tspline& tsp, unsigned& segX, unsigned& segY) {
	//1. vars
	easy3d::SurfaceMesh new_mesh;
	auto new_mesh_quad_param = new easy3d::SurfaceMesh();
	new_mesh.clear(), new_mesh_quad_param->clear();
	
	//2. create the quad tsp as the quad param 
	tspline::TsplineCreator param_quad_creator;
	auto quad_tsp = std::make_shared<tspline::Tspline>();
	param_quad_creator.CreatePlaneXY(*quad_tsp, 0.0, 0.0, 0.0, 1.0, 1.0, segX, segY);
	quad_tsp->update_vertex_ids();
	//convert the tsp to surfacemesh
	convert_tsp_2_surfacemesh(*quad_tsp, *new_mesh_quad_param);
	
	//3.compute result
	for (auto fit : new_mesh_quad_param->faces()) {
		std::vector<easy3d::SurfaceMesh::Vertex> face_vertices;
		face_vertices.clear();
		std::map<easy3d::SurfaceMesh::Vertex, easy3d::SurfaceMesh::Vertex> global_vertex_map;
		global_vertex_map.clear();
		if (fit.is_valid()) {
			for (auto he : new_mesh_quad_param->halfedges(fit)) {
				if (he.is_valid()) {
					auto v = new_mesh_quad_param->target(he);
					auto v_it = global_vertex_map.find(v);
					if (v_it == global_vertex_map.end()) {
						easy3d::dvec3 param_pos = static_cast<easy3d::dvec3>(new_mesh_quad_param->position(v));
						Eigen::Vector3d result = tsp.evaluate(param_pos.x, param_pos.y);
						float x = static_cast<float>(result.x());
						float y = static_cast<float>(result.y());
						float z = static_cast<float>(result.z());
						easy3d::SurfaceMesh::Vertex new_v = new_mesh.add_vertex(easy3d::vec3(x, y, z));
						global_vertex_map[v] = new_v;
						face_vertices.push_back(new_v);
					}
					else {
						face_vertices.push_back(v_it->second);
					}
				}
			}
		}
		if (face_vertices.size() >= 4) {
			new_mesh.add_face(face_vertices);
		}
	}

	std::shared_ptr<easy3d::SurfaceMesh> new_mesh_ptr = std::make_shared<easy3d::SurfaceMesh>(new_mesh);
	if (mesh_results_quad.size() == 0) {
		std::vector<std::shared_ptr<easy3d::SurfaceMesh>> mesh_ptrs;
		mesh_ptrs.push_back(new_mesh_ptr);
		mesh_results_quad.push_back(mesh_ptrs);
	}
	else {
	
		if (mesh_idx >= 0 && mesh_idx < mesh_results_quad.size()) {
			mesh_results_quad[mesh_idx].push_back(new_mesh_ptr);
			std::cout << "mesh_results_quad[" << mesh_idx << "] size: " << mesh_results_quad[mesh_idx].size() << std::endl;
		} else {
			std::cout << "ERROR: Invalid mesh_idx in compute_result_quad_mesh: " << mesh_idx << std::endl;
		}
		std::cout << "mesh_results_quad total size " << mesh_results_quad.size() << std::endl;
	}

}

//========================================== Global LSM ==========================================
void MyImGui::Global_LSM(double& epsilon, unsigned& min_point_count) {
	if (mesh_idx == -1 || mesh_tsps.size() == 0) {
		return;
	}
	else {
		auto& tsps = mesh_tsps[mesh_idx];
		if (tsps.size() == 0) {
			std::cout << "ERROR: haven't add orignal TMesh." << std::endl;
			return;
		}
		else {
			//1. pre compute
			std::vector<easy3d::dvec3> mesh;
			std::vector<easy3d::dvec3> mesh_param;
			auto mesh_ = meshes[mesh_idx];
			auto mesh_param_ = mesh_params[mesh_idx];
			convert_vec3_2_dvec3(mesh_.get(), mesh);
			convert_vec3_2_dvec3(mesh_param_.get(), mesh_param);

			//2.
			int refined_size = tsps.size();
			auto tsp = tsps[refined_size - 1];
			tspline::Tspline evaluate_tsp = *tsp;//tsp1
			tspline::Tspline refine_tsp = *tsp;//tsp2

			std::cout << "Start iteration: " << std::endl;

			std::vector<double> all_tface_error;
			std::vector<int> all_tface_point_count;

			update_n_vertex_in_tfaces(evaluate_tsp, mesh_param);//only update every face point count
			evaluate_all_tface_error(evaluate_tsp, mesh, mesh_param, min_point_count, all_tface_error);//only compute error
			update_all_tface_error(evaluate_tsp, all_tface_error);//only update the face error
			update_n_vertex_in_tfaces(evaluate_tsp, all_tface_point_count);

			//1.3 collect all refined faces's boundary box
			std::cout << "[DEBUG] Collecting boundary boxes for refined faces..." << std::endl;
			std::vector<Eigen::Vector4d> all_bb;
			int fit_count(0);
			int valid_faces = 0;
			int refined_faces = 0;

			for (auto fit = evaluate_tsp.faces_begin(); fit != evaluate_tsp.faces_end(); fit++, fit_count++) {
				if (!fit->is_unbounded() && fit->has_outer_ccb()) {
					valid_faces++;
					//safe, makesure that all fit is valid.
					if (all_tface_error[fit_count] > epsilon && all_tface_point_count[fit_count] > min_point_count) {
						refined_faces++;
						Eigen::Vector4d bb;
						evaluate_tsp.face_bounding_box(fit, bb);
						all_bb.push_back(bb);
						std::cout << "[DEBUG] Face " << fit_count << " needs refinement - error: " << all_tface_error[fit_count]
							<< ", point_count: " << all_tface_point_count[fit_count] << std::endl;
					}
				}
			}
			std::cout << "[DEBUG] Valid faces: " << valid_faces << ", faces needing refinement: " << refined_faces << std::endl;
			std::cout << "[DEBUG] all_bb size is " << all_bb.size() << std::endl;

			//1.4 use boundary box to find the refined face in next_tsp_.
			std::cout << "[DEBUG] Searching for faces to refine in next_tsp_..." << std::endl;
			int faces_checked = 0;
			int faces_refined = 0;

			for (auto fit = refine_tsp.faces_begin(); fit != refine_tsp.faces_end(); fit++) {
				if (!fit->is_unbounded() && fit->has_outer_ccb()) {
					faces_checked++;
					Eigen::Vector4d bb;
					refine_tsp.face_bounding_box(fit, bb);
					auto bb_found = std::find(all_bb.begin(), all_bb.end(), bb);
					if (bb_found != all_bb.end()) {
						std::cout << "[DEBUG] Found face to refine, performing refinement..." << std::endl;
						faces_refined++;
						//fint the target fit.
						refine_tsp.refine(fit, false, false);
						fit = refine_tsp.faces_begin();//reset the fit, makesure that next search is valid.
					}
				}
			}
			std::cout << "[DEBUG] Checked " << faces_checked << " faces, refined " << faces_refined << " faces" << std::endl;

			//keep topology.
			std::cout << "[DEBUG] Updating topology..." << std::endl;
			refine_tsp.insert_missing_edges();
			refine_tsp.update_knot_vectors();
			std::cout << "[DEBUG] Topology updated successfully" << std::endl;

			//1.5 recompute the controlpoints and add tsp and result mesh into models.
			std::cout << "[DEBUG] Computing control points..." << std::endl;

			// Output last_tsp_ control vertices information
			std::cout << "[DEBUG] last_tsp_ control vertices (before computation):" << std::endl;
			for (tspline::Tspline::Vertex_const_iterator v = evaluate_tsp.vertices_begin(); v != evaluate_tsp.vertices_end(); v++) {
				const tspline::TVertex& vext = v->data();
				int cp_id = vext.id;
				auto cp = vext.GetCP();
				std::cout << "[DEBUG] last_tsp_ vertex " << cp_id << ": (" << cp.x() << ", " << cp.y() << ", " << cp.z() << ")" << std::endl;
			}

			compute_control_points(refine_tsp, mesh, mesh_param);

			// Output next_tsp_ control vertices information (after computation)
			std::cout << "[DEBUG] next_tsp_ control vertices (after computation):" << std::endl;
			for (tspline::Tspline::Vertex_const_iterator v = refine_tsp.vertices_begin(); v != refine_tsp.vertices_end(); v++) {
				const tspline::TVertex& vext = v->data();
				int cp_id = vext.id;
				auto cp = vext.GetCP();
				std::cout << "[DEBUG] next_tsp_ vertex " << cp_id << ": (" << cp.x() << ", " << cp.y() << ", " << cp.z() << ")" << std::endl;
			}

			std::cout << "[DEBUG] Control points computed successfully" << std::endl;

			std::cout << "[DEBUG] Adding refine result..." << std::endl;
			std::cout << "[DEBUG] Before adding - mesh_tsps[" << mesh_idx << "] size: " << mesh_tsps[mesh_idx].size() << std::endl;
			std::cout << "[DEBUG] Before adding - mesh_results_quad[" << mesh_idx << "] size: " << (mesh_idx < mesh_results_quad.size() ? mesh_results_quad[mesh_idx].size() : -1) << std::endl;
			
			tsps.push_back(std::make_shared<tspline::Tspline>(refine_tsp));
			compute_result_mesh(refine_tsp, mesh_param_.get());
			unsigned segX = 25, segY = 25;
			compute_result_quad_mesh(refine_tsp, segX, segY);
			
			std::cout << "[DEBUG] After adding - mesh_tsps[" << mesh_idx << "] size: " << mesh_tsps[mesh_idx].size() << std::endl;
			std::cout << "[DEBUG] After adding - mesh_results_quad[" << mesh_idx << "] size: " << (mesh_idx < mesh_results_quad.size() ? mesh_results_quad[mesh_idx].size() : -1) << std::endl;
			std::cout << "result_mesh_quad total size " << mesh_results_quad.size() << std::endl;
			std::cout << "[DEBUG] Refine result added successfully" << std::endl;


			std::cout << "Finished iteration: " << std::endl;
		}
	}
}

//============================================== Auxiliary functions ==========================
void MyImGui::evaluate_single_tface_error(tspline::Tspline& tsp,
	tspline::Tspline::Face_iterator& fit,
	std::vector<easy3d::dvec3>& mesh,
	std::vector<easy3d::dvec3>& mesh_param,
	unsigned min_point_count,
	double& face_error) {
	face_error = 0.0;
	//1. pre compute
	if (!fit->is_unbounded() && fit->has_outer_ccb()) {
		std::vector<int> params;
		get_params_in_face(tsp, fit, mesh_param, params);
		if (params.size() >= min_point_count) {
			double temp_all_error(0.0);
			for (int i = 0; i < params.size(); i++) {
				Eigen::Vector3d tsp_p = tsp.evaluate(mesh_param[params[i]].x, mesh_param[params[i]].y);
				Eigen::Vector3d pat_p(mesh[params[i]].x, mesh[params[i]].y, mesh[params[i]].z);
				double single_error(0.0);
				evaluate_dis(tsp_p, pat_p, single_error);
				temp_all_error += single_error;
			}
			face_error = temp_all_error / (double)(params.size());
		}
	}
}
void MyImGui::evaluate_all_tface_error(tspline::Tspline& tsp,
	std::vector<easy3d::dvec3>& mesh,
	std::vector<easy3d::dvec3>& mesh_param,
	unsigned min_point_count,
	std::vector<double>& all_face_error) {
	all_face_error.clear();
	int valid_face_count = 0;
	for (auto fit = tsp.faces_begin(); fit != tsp.faces_end(); fit++) {
		if (!fit->is_unbounded() && fit->has_outer_ccb()) {
			valid_face_count++;
		}
	}
	all_face_error.resize(valid_face_count);

	int face_index = 0;
	for (auto fit = tsp.faces_begin(); fit != tsp.faces_end(); fit++) {
		if (!fit->is_unbounded() && fit->has_outer_ccb()) {
			// std::cout << "Evaluating face " << face_index << "/" << valid_face_count - 1 << ": " << std::endl;
			bool isRefined(false);
			double error(0.0);
			evaluate_single_tface_error(tsp, fit, mesh, mesh_param, min_point_count, error);
			all_face_error[face_index] = error;
			face_index++;
		}
		else {
			// std::cout << "Found the unbounded face or face without outer contour, skip face" << std::endl;
		}
	}
}


//============================================================ inline functions ==============================================
inline void MyImGui::evaluate_dis(Eigen::Vector3d& p1, Eigen::Vector3d& p2, double& result) {
	double d = (p1 - p2).norm();
	result = d;
}

inline void MyImGui::get_params_in_face(tspline::Tspline& tsp, tspline::Tspline::Face_iterator& fit, std::vector<easy3d::dvec3>& params, std::vector<int>& params_in_face) {
	params_in_face.clear();
	if (params.size() == 0) {
		return;
	}

	for (tspline::Tspline::Face_iterator fit_ = tsp.faces_begin(); fit_ != tsp.faces_end(); fit_++) {
		if (fit == fit_) {
			Eigen::Vector4d bb;
			tsp.face_bounding_box(fit_, bb);
			double min_x = bb(0), min_y = bb(2);
			double max_x = bb(1), max_y = bb(3);

			for (int i = 0; i < params.size(); i++) {
				double param_x = params[i].x;
				double param_y = params[i].y;
				if (param_x >= min_x && param_x <= max_x && param_y >= min_y && param_y <= max_y) {
					params_in_face.push_back(i);
				}
			}
		}
	}
}

inline void MyImGui::update_n_vertex_in_tfaces(tspline::Tspline& tsp, std::vector<int>& n_param) {
	n_param.clear();
	for (auto fit = tsp.faces_begin(); fit != tsp.faces_end(); fit++) {
		if (!fit->is_unbounded() && fit->has_outer_ccb()) {
			n_param.push_back(fit->data().point_count);
		}
	}
}
inline void MyImGui::update_n_vertex_in_tfaces(tspline::Tspline& tsp, std::vector<easy3d::dvec3>& param_double) {
	for (tspline::Tspline::Face_iterator fit = tsp.faces_begin(); fit != tsp.faces_end(); fit++) {
		if (!fit->is_unbounded() && fit->has_outer_ccb()) {
			int point_count_ = 0; // Calculate parameter point count for each face separately
			Eigen::Vector4d bb;
			tsp.face_bounding_box(fit, bb);
			double min_x = bb(0), min_y = bb(2);
			double max_x = bb(1), max_y = bb(3);
			for (int i = 0; i < param_double.size(); i++) {
				double param_x = param_double[i].x;
				double param_y = param_double[i].y;
				if (param_x >= min_x && param_x <= max_x && param_y >= min_y && param_y <= max_y) {
					point_count_++;
				}
			}
			fit->data().point_count = point_count_;
		}
	}
}
inline void MyImGui::update_all_tface_error(tspline::Tspline& tsp, std::vector<double>& all_tface_error) {
	int i = 0;
	for (auto fit = tsp.faces_begin(); fit != tsp.faces_end(); fit++) {
		if (!fit->is_unbounded() && fit->has_outer_ccb()) {
			tspline::TFace& fext = fit->data();
			fext.error_sqr = all_tface_error[i];
			i++;
		}
	}
}

inline easy3d::SurfaceMesh::Face MyImGui::get_mesh_face_by_idx(easy3d::SurfaceMesh* mesh, int face_idx)const {
	int curr_face_idx = 0;
	for (auto f : mesh->faces()) {
		if (curr_face_idx == face_idx) return f;
		curr_face_idx++;
	}
	return easy3d::SurfaceMesh::Face{};
}
inline easy3d::SurfaceMesh::Vertex MyImGui::get_mesh_vertex_by_idx(easy3d::SurfaceMesh* mesh, int vertex_idx)const {
	int curr_vertex_idx(0);
	for (auto v : mesh->vertices()) {
		if (curr_vertex_idx == vertex_idx) return v;
		curr_vertex_idx++;
	}
	return easy3d::SurfaceMesh::Vertex{};
}
