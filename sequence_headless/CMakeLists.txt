cmake_minimum_required(VERSION 3.19)

add_executable(sequence_headless
  sequence_headless.cpp
  ../fast_event_detector.cpp
  ../metadata_loader.cpp
  ../onnx_classifier.cpp
  ../daq_trigger.cpp
  ../qt_hama_gui/pipeline_runner.cpp
)

target_include_directories(sequence_headless PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/../qt_hama_gui
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(sequence_headless PRIVATE
  ${ONNXRUNTIME_LIB}
  ${OpenCV_LIBS}
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
      add_custom_command(TARGET sequence_headless POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${onnx_dll}" $<TARGET_FILE_DIR:sequence_headless>
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
    target_compile_definitions(sequence_headless PRIVATE HAVE_NIDAQMX=1)
    target_include_directories(sequence_headless PRIVATE ${NIDAQMX_INCLUDE_DIR})
    target_link_libraries(sequence_headless PRIVATE ${NIDAQMX_LIBRARY})
  else()
    message(WARNING "NI-DAQmx not found; building with stub trigger.")
  endif()
endif()
