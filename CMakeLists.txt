cmake_minimum_required(VERSION 3.19)
project(PointCloudProcessor LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)

qt_standard_project_setup()

qt_add_executable(PointCloudProcessor
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    src/core/MmapPlyReader.cpp
    src/core/MmapPlyReader.h
    src/core/Octree.cpp
    src/core/Octree.h
    src/core/PointCloudLoader.cpp
    src/core/PointCloudLoader.h
    src/core/Types.h
    src/workers/LoadWorker.cpp
    src/workers/LoadWorker.h
    src/ui/PointCloudViewerWidget.cpp
    src/ui/PointCloudViewerWidget.h
)

target_include_directories(PointCloudProcessor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_features(PointCloudProcessor PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(PointCloudProcessor PRIVATE /bigobj)
    target_compile_definitions(PointCloudProcessor PRIVATE NOMINMAX)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(PointCloudProcessor PRIVATE -Wa,-mbig-obj)
endif()

target_link_libraries(PointCloudProcessor
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt6::OpenGLWidgets
        Open3D::Open3D
        Eigen3::Eigen
)

install(TARGETS PointCloudProcessor
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET PointCloudProcessor
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})