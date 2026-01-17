cmake_minimum_required(VERSION 3.19)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(VCPKG_ROOT "C:/vcpkg" CACHE PATH "Path to vcpkg")
  if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    if (EXISTS "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
      set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
    endif()
  endif()
  project(qt_hama_gui LANGUAGES CXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(DCAM_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../python_pipeline/Hamamatsu_DCAMSDK4_v25056964/dcamsdk4" CACHE PATH "Path to Hamamatsu DCAM SDK (dcamsdk4)")
set(ONNXRUNTIME_DIR "" CACHE PATH "Path to ONNX Runtime")
set(NIDAQMX_DIR "" CACHE PATH "Path to NI-DAQmx (optional)")
option(ENABLE_NIDAQMX "Enable NI-DAQmx trigger support" ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
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

add_executable(qt_hama_gui
  main.cpp
  dcam_controller.cpp
  frame_grabber.cpp
  pipeline_runner.cpp
  ../cli_runner.cpp
  ../dcam_camera.cpp
  ../event_detector.cpp
  ../daq_trigger.cpp
  ../fast_event_detector.cpp
  ../metadata_loader.cpp
  ../onnx_classifier.cpp
)

add_executable(daq_test_cli
  ../daq_test_cli.cpp
  ../daq_trigger.cpp
)

if (WIN32)
  set_target_properties(qt_hama_gui PROPERTIES WIN32_EXECUTABLE ON)
  if (TARGET Qt6::EntryPoint)
    target_link_libraries(qt_hama_gui PRIVATE Qt6::EntryPoint)
  elseif (TARGET Qt6::WinMain)
    target_link_libraries(qt_hama_gui PRIVATE Qt6::WinMain)
  endif()
endif()

set_target_properties(qt_hama_gui PROPERTIES
  OUTPUT_NAME droplet_pipeline
)

target_include_directories(qt_hama_gui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${DCAM_SDK_DIR}/inc
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(qt_hama_gui PRIVATE
  ${DCAM_SDK_DIR}/lib/win64
)

target_link_libraries(qt_hama_gui PRIVATE
  Qt6::Widgets
  dcamapi
  ${ONNXRUNTIME_LIB}
  ${OpenCV_LIBS}
)

target_include_directories(daq_test_cli PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

if (ONNXRUNTIME_DIR)
  set(ONNX_DLLS
    "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll"
    "${ONNXRUNTIME_DIR}/lib/onnxruntime_providers_shared.dll"
    "${ONNXRUNTIME_DIR}/lib/onnxruntime_providers_cuda.dll"
    "${ONNXRUNTIME_DIR}/lib/onnxruntime_providers_tensorrt.dll"
  )
  foreach(onnx_dll IN LISTS ONNX_DLLS)
    if (EXISTS "${onnx_dll}")
      add_custom_command(TARGET qt_hama_gui POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${onnx_dll}" $<TARGET_FILE_DIR:qt_hama_gui>
      )
    endif()
  endforeach()
endif()

if (ENABLE_NIDAQMX)
  find_path(NIDAQMX_INCLUDE_DIR NIDAQmx.h
    PATHS
      "${NIDAQMX_DIR}/include"
      "C:/Program Files/National Instruments/NI-DAQ/DAQmx ANSI C Dev/include"
      "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/include"
  )
  find_library(NIDAQMX_LIBRARY NAMES NIDAQmx
    PATHS
      "${NIDAQMX_DIR}/lib/x64/msvc"
      "${NIDAQMX_DIR}/lib/msvc"
      "C:/Program Files/National Instruments/NI-DAQ/DAQmx ANSI C Dev/lib/msvc"
      "C:/Program Files (x86)/National Instruments/Shared/ExternalCompilerSupport/C/lib64/msvc"
      "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/lib/x64/msvc"
      "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/lib/msvc"
  )
  if (NIDAQMX_INCLUDE_DIR AND NIDAQMX_LIBRARY)
    foreach(daq_target IN ITEMS qt_hama_gui daq_test_cli)
      target_compile_definitions(${daq_target} PRIVATE HAVE_NIDAQMX=1)
      target_include_directories(${daq_target} PRIVATE ${NIDAQMX_INCLUDE_DIR})
      target_link_libraries(${daq_target} PRIVATE ${NIDAQMX_LIBRARY})
    endforeach()
  else()
    message(WARNING "NI-DAQmx not found; building with stub trigger.")
  endif()
endif()
