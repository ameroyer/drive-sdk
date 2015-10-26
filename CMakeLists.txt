cmake_minimum_required(VERSION 2.8)

project(drivekit)

set(CMAKE_MODULE_PATH ${drivekit_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

# Set the default install path to be './build/dist'
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) 
    set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/dist CACHE PATH "drive-sdk install prefix" FORCE) 
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) 

OPTION( BUILD_EXAMPLES                "Build example apps"                OFF )
OPTION( BUILD_PROJECT                "Build code for cvml project"                OFF )

SET (CMAKE_C_FLAGS                "-Wall -std=c99")
include(MacroOutOfSourceBuild)
include(TargetDistclean)

macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build.")
 
add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY include/ankidrive DESTINATION include)

if (BUILD_EXAMPLES)
    add_subdirectory(deps/bzle)
    add_subdirectory(examples)
endif ()

### Compile only the files added for the project
if (BUILD_PROJECT)
    if (NOT BUILD_EXAMPLES)
      add_subdirectory(deps/bzle)
    ENDIF()
    SET (CMAKE_C_FLAGS      "")
    include(FindGLIB2)
    include(FindReadline)
    include_directories(${drivekit_SOURCE_DIR}/include
                    ${bzle_SOURCE_DIR}/include
                    ${GLIB2_INCLUDE_DIRS}
                    ${READLINE_INCLUDE_DIR}
		    /usr/include/flycapture
                    )
    set(Project_SOURCES
		examples/simple-c-interface/anki-simplified.h
		examples/simple-c-interface/anki-simplified.c
		CV/get_camera.c
		)
    add_executable(main main.c ${Project_SOURCES})
    target_link_libraries(main
                    ankidrive
                    bzle
                    ${GLIB2_LIBRARIES}
                    ${READLINE_LIBRARY}
		    flycapture
		    pthread
		    )
    install(TARGETS main 
	RUNTIME DESTINATION bin
    )
endif ()

# CTest is flaky - Create a target to run our test suite directly
add_custom_target(test COMMAND ${PROJECT_BINARY_DIR}/test/Test)
