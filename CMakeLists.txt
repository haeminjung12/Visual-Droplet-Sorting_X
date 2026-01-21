cmake_minimum_required(VERSION 3.19)

set(VCPKG_ROOT "C:/vcpkg" CACHE PATH "Path to vcpkg")
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if (EXISTS "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
  endif()
endif()

project(droplet_pipeline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DCAM_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../python_pipeline/Hamamatsu_DCAMSDK4_v25056964/dcamsdk4" CACHE PATH "Path to Hamamatsu DCAM SDK (dcamsdk4)")
set(ONNXRUNTIME_DIR "" CACHE PATH "Path to ONNX Runtime")
set(NIDAQMX_DIR "" CACHE PATH "Path to NI-DAQmx (optional)")

option(ENABLE_NIDAQMX "Enable NI-DAQmx trigger support" ON)
option(BUILD_QT_GUI "Build Qt GUI app" ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

if (ONNXRUNTIME_DIR)
  if (NOT ONNXRUNTIME_INCLUDE_DIR)
    find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h PATHS "${ONNXRUNTIME_DIR}/include" NO_DEFAULT_PATH)
  endif()
  if (NOT ONNXRUNTIME_LIB)
    find_library(ONNXRUNTIME_LIB onnxruntime PATHS "${ONNXRUNTIME_DIR}/lib" "${ONNXRUNTIME_DIR}/lib64" NO_DEFAULT_PATH)
  endif()
else()
  if (NOT ONNXRUNTIME_INCLUDE_DIR)
    find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h)
  endif()
  if (NOT ONNXRUNTIME_LIB)
    find_library(ONNXRUNTIME_LIB onnxruntime)
  endif()
endif()
if (NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "ONNX Runtime not found. Set ONNXRUNTIME_DIR to a valid install path.")
endif()

set(DCAM_INCLUDE_DIR "${DCAM_SDK_DIR}/inc")
set(DCAM_LIB_DIR "${DCAM_SDK_DIR}/lib/win64")

if (BUILD_QT_GUI)
  add_subdirectory(qt_hama_gui)
endif()

add_subdirectory(sequence_headless)
