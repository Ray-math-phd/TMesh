#include <QApplication>
#include <QSurfaceFormat>
#include <QStyleFactory>
#include <QDir>
#include <easy3d/util/initializer.h>
#include <easy3d/util/resource.h>
#include <easy3d/renderer/camera.h>

#include <iostream>

#include "MyImGui.h"

int main(int argc, char* argv[]) {

#ifdef PROJECT_RESOURCE_DIR
    easy3d::initialize(false, false, false, PROJECT_RESOURCE_DIR);
#else
    easy3d::initialize();
#endif


    MyImGui viewer("my ImGui");

    viewer.camera()->setViewDirection(easy3d::vec3(0, 0, -1));
    viewer.camera()->setUpVector(easy3d::vec3(0, 1, 0));


    std::string model_file = easy3d::resource::directory() + "/data/easy3d.ply";
    if (!viewer.add_model(model_file)) {
        std::cerr << "Failed to load default model: " << model_file << std::endl;
        std::cerr << "Please ensure model files are placed in resources/data/ directory" << std::endl;
    }

 
    return viewer.run();
}