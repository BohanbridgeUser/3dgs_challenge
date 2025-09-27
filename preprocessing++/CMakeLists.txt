cmake_minimum_required(VERSION 3.10)
project(PREPROCESS LANGUAGES CXX)

# -------------------- 构建类型 --------------------
# 默认 Release，如果用户指定则使用用户配置
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------- 编译器选项 --------------------
if(MSVC)
    add_compile_options(/W4 /WX /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# -------------------- 第三方库 --------------------
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV_FOUND found")
endif()
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# 尝试找到 pybind11，如果没找到不会报错
find_package(pybind11 REQUIRED)
if(pybind11_FOUND)
    message(STATUS "pybind11 found, building Python module py_preprocess")
else()
    message(STATUS "pybind11 not found")
endif()

# -------------------- 源文件 --------------------
set(SRC_LIST
    main.cpp
    Pipeline.cpp
)

# # -------------------- 可执行文件 --------------------
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

# -------------------- Python 模块（可选） --------------------
if(pybind11_FOUND)
    message(STATUS "pybind11 found, building Python module py_preprocess")
    
    pybind11_add_module(py_preprocess
        pybind_module.cpp
        Pipeline.cpp
    )

    target_include_directories(py_preprocess PRIVATE
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(py_preprocess PRIVATE
        ${OpenCV_LIBS}
        Eigen3::Eigen
    )

    if(OpenMP_CXX_FOUND)
        target_link_libraries(py_preprocess PRIVATE OpenMP::OpenMP_CXX)
    endif()

    IF(UNIX)
        execute_process(
            COMMAND which python
            OUTPUT_VARIABLE Python3_EXECUTABLE
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        set(Python3_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python interpreter")
        
        find_package(Python3 COMPONENTS Interpreter REQUIRED)
        message(STATUS "Python version: ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")

        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
            OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        message(STATUS "Python site-packages: ${PYTHON_SITE_PACKAGES}")

        add_custom_command(TARGET py_preprocess POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:py_preprocess> ${PYTHON_SITE_PACKAGES}
            COMMENT "Copying py_preprocess.so to Python site-packages: ${PYTHON_SITE_PACKAGES}"
        )
    ENDIF()

else()
    message(WARNING "pybind11 not found, only building C++ executable")
endif()


