cmake_minimum_required(VERSION 3.10)
project("TMCC")

# 添加：设置CMake策略 - 恢复使用FindBoost.cmake


# ================ 基础配置 ================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 添加：禁用Qt调试信息
if(MSVC)
    # 使用Z7格式（不包含源码路径）
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Z7")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Z7")
    
    # 禁用Qt调试输出
    add_compile_definitions(
        QT_NO_DEBUG_OUTPUT
        QT_NO_WARNING_OUTPUT
        QT_NO_INFO_OUTPUT
    )
endif()

# ================ 库根路径 ================
set(LIBS_ROOT "${CMAKE_SOURCE_DIR}/lib")


#================= Sources ========================
set(PROJECT_RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")
# 确保resources目录存在
if(NOT EXISTS "${PROJECT_RESOURCE_DIR}")
    file(MAKE_DIRECTORY "${PROJECT_RESOURCE_DIR}")
endif()

# ================ QT 配置 ================
set(QT_ROOT "C:/Qt/5.15.2/msvc2019_64")
set(CMAKE_PREFIX_PATH ${QT_ROOT})
find_package(Qt5 5.15.2 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

#================= Boost ==================
set(BOOST_ROOT "${LIBS_ROOT}/boost_1_78_0")
set(BOOST_INCLUDEDIR "${BOOST_ROOT}/boost")
set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib64-msvc-14.1")
set(BOOST_LIBRARIES "${BOOST_ROOT}/lib64-msvc-14.1")

# 设置Boost查找变量
set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost 1.78.0 REQUIRED system)
if(Boost_FOUND)
    message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost library dir: ${Boost_LIBRARY_DIRS}")
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
    link_directories(${BOOST_LIBRARIES})
else()
    message(FATAL_ERROR "Boost not found!")
endif()



# ================ Easy3D 手动配置 ================
set(EASY3D_ROOT "${LIBS_ROOT}/Easy3D")  # 假设这是您安装Easy3D的路径

# 添加Easy3D头文件
include_directories(${EASY3D_ROOT}/include)
include_directories(${EASY3D_ROOT}/3rd_party)
include_directories(${EASY3D_ROOT}/3rd_party/imgui)
include_directories(${EASY3D_ROOT}/3rd_party/glfw/include)

# 添加Easy3D库目录
link_directories(${EASY3D_ROOT}/lib)
link_directories(${EASY3D_ROOT}/build/lib/Release)

# ================ CGAL & Boost 配置 ================
set(CGAL_DIR "${LIBS_ROOT}/CGAL-5.3")
find_package(CGAL 5.3 REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})

# 添加CGAL参数化模块头文件路径
include_directories(${CGAL_DIR}/include)

# ================ Eigen 配置 ================
set(EIGEN3_DIR "${LIBS_ROOT}/eigen")
include_directories(${EIGEN3_DIR})

# ================ 源文件配置 ================
set(SOURCE_FILES
    src/main.cpp
    src/MyImGui.cpp
    src/TMeshOptimizer.cpp
    src/MyParam.cpp
    src/Cox.cpp
    src/Tspline.cpp
    src/Tspline_operators.cpp
    src/TsplineCreator.cpp
    src/Types.cpp
    src/MyImGui_Operators.cpp
    
    src/TSkeleton.cpp
    src/TSkeletonPolygonFace.cpp
    )

set(HEADER_FILES
    include/MyImGui.h
    include/TMeshOptimizer.h
    include/MyParam.h
    include/Cox.h
    include/Tspline.h
    include/Types.h
    include/TsplineCreator.h
    include/Math.hpp
    include/SMath.hpp
    include/TSkeletonTypes.h
    include/TSkeleton.h
    include/TSkeletonPolygonFace.h
    )

set(UI_FILES
    )

# ================ 主可执行文件 ================
add_executable(${PROJECT_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${UI_FILES}
)

# 添加：配置Qt符号文件（移动到目标创建之后）
if(WIN32 AND MSVC)
    # 设置Qt符号文件路径
    set(QT_SYMBOL_PATH "${QT_ROOT}/lib")
    
    # 禁用Qt源码调试
    set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_SYMBOL_PATH "${QT_SYMBOL_PATH}")
    
    # 设置调试器选项
    set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_ENVIRONMENT 
        "QT_LOGGING_RULES=*.debug=false;*.info=false;*.warning=false")
    
    # 设置调试器属性
    set_target_properties(${PROJECT_NAME} PROPERTIES
        VS_DEBUGGER_SYMBOL_PATH ""
        VS_DEBUGGER_SOURCE_PATH ""
    )
endif()

# ================ 包含目录 ================
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
    "${LIBS_ROOT}/ThirdParty/include"
    ${CGAL_INCLUDE_DIRS}
    ${EIGEN3_DIR}
    ${EASY3D_ROOT}/include  # 确保Easy3D头文件被包含
    ${EASY3D_ROOT}/3rd_party  # 确保Easy3D第三方库头文件被包含
    ${EASY3D_ROOT}/3rd_party/imgui  # 确保ImGui头文件被包含
    ${EASY3D_ROOT}/3rd_party/glfw/include  # 确保GLFW头文件被包含
)

# ================ 链接库 ================
target_link_libraries(${PROJECT_NAME}
    # 手动链接Easy3D库 - 修正库名称大小写
    easy3d_core.lib
    easy3d_viewer.lib
    easy3d_renderer.lib
    easy3d_util.lib
    easy3d_fileio.lib
    easy3d_algo.lib
    easy3d_gui.lib
    easy3d_kdtree.lib
    # 第三方库
    3rd_glfw.lib
    3rd_imgui.lib
    3rd_kdtree.lib
    3rd_lastools.lib
    3rd_opcode.lib
    3rd_poisson.lib
    3rd_polypartition.lib
    3rd_ransac.lib
    3rd_rply.lib
    3rd_tetgen.lib
    3rd_triangle.lib
    3rd_glutess.lib
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    CGAL::CGAL
    CGAL::CGAL_Core
    ${Boost_LIBRARIES}
    opengl32.lib
    glu32.lib
)

# ================ Windows 部署 ================
if(WIN32 AND MSVC)
    # Qt部署 - 修复复制命令
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${QT_ROOT}/bin/Qt5Core.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${QT_ROOT}/bin/Qt5Gui.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${QT_ROOT}/bin/Qt5Widgets.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${QT_ROOT}/bin/Qt5OpenGL.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E make_directory
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${QT_ROOT}/plugins/platforms/qwindows.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>/platforms/
    )
    
    # Easy3D DLL部署 - 修复复制命令
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_core.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_viewer.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_renderer.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_util.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_fileio.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_algo.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_gui.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/easy3d_kdtree.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
    
    # 复制Easy3D第三方库DLL
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_glfw.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_imgui.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_kdtree.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_lastools.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_opcode.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_poisson.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_polypartition.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_ransac.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_rply.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_tetgen.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_triangle.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${EASY3D_ROOT}/bin/3rd_glutess.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
    
    # CGAL DLL部署
    if(EXISTS "${CGAL_DIR}/auxiliary/gmp/lib/gmp-10.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CGAL_DIR}/auxiliary/gmp/lib/gmp-10.dll"
                $<TARGET_FILE_DIR:${PROJECT_NAME}>
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CGAL_DIR}/auxiliary/gmp/lib/mpfr-6.dll"
                $<TARGET_FILE_DIR:${PROJECT_NAME}>
        )
    endif()
    
    # 预处理器定义
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        _SCL_SECURE_NO_WARNINGS
        BOOST_ALL_NO_LIB
        CGAL_DO_NOT_USE_BOOST_AUTO_LINK
        Easy3D_RESOURCE_DIR="${PROJECT_RESOURCE_DIR}"
        PROJECT_RESOURCE_DIR="${PROJECT_RESOURCE_DIR}"
        ELPP_FEATURE_ALL
        ELPP_STL_LOGGING
        ELPP_THREAD_SAFE
        ELPP_NO_DEFAULT_LOG_FILE
        ELPP_DISABLE_DEFAULT_CRASH_HANDLING
        # 添加：禁用Qt调试输出
        QT_NO_DEBUG_OUTPUT
        QT_NO_WARNING_OUTPUT
        QT_NO_INFO_OUTPUT
    )
    
    # 设置工作目录和资源路径
    set_target_properties(${PROJECT_NAME} PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        VS_DEBUGGER_ENVIRONMENT "PATH=${EASY3D_ROOT}/bin;$ENV{PATH}"
    )
    
    # 复制项目资源文件
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${PROJECT_RESOURCE_DIR}"
            "${CMAKE_CURRENT_BINARY_DIR}/resources"
    )
    
    # 复制Easy3D资源文件（如果项目resources目录为空）
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${EASY3D_ROOT}/resources"
            "${CMAKE_CURRENT_BINARY_DIR}/resources"
    )
endif()