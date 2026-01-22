# Copyright (c) Prophesee S.A.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.

# Compile the apps to check they compile
set(standalone_app
    evs_2ms_logger
)

foreach(sample ${standalone_app})
    add_executable(${sample} ${sample}/${sample}.cpp)

    target_link_libraries(${sample}
        PRIVATE
            MetavisionSDK::core
            MetavisionSDK::stream
            opencv_highgui
            opencv_imgproc
    )

    if(WIN32 AND sample STREQUAL "evs_2ms_logger")
        add_custom_command(TARGET ${sample} POST_BUILD
            COMMAND ${CMAKE_COMMAND}
                -DPLUGIN_DIR="${CMAKE_BINARY_DIR}/lib/metavision/hal/plugins"
                -DSDK_DLL_DIR="${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
                -DTARGET_DIR="$<TARGET_FILE_DIR:${sample}>"
                -P "${CMAKE_SOURCE_DIR}/cmake/copy_evs_2ms_logger_runtime.cmake"
            COMMENT "Copying EVS 2ms logger runtime DLLs and plugins to output directory"
        )
    endif()
endforeach(sample)
