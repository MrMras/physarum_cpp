#Specify the version being used aswell as the language
cmake_minimum_required(VERSION 3.19)

#Name your project here
project(phys LANGUAGES CXX)


#set the module directory
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
set(CMAKE_CXX_STANDARD 20)

#find_package(OpenGL REQUIRED)
#find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
#find_package(imgui CONFIG REQUIRED core glfw-binding opengl3-binding)
#find_package(GLEW REQUIRED)
find_package(TIRA REQUIRED)


#build the executable in the binary directory on MS Visual Studio
if ( MSVC )
	SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
	SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")
	SET( LIBRARY_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
	SET( LIBRARY_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	add_definitions(-D_SCL_SECURE_NO_WARNINGS)
endif ( MSVC )

#configure_file(Roboto-Medium.ttf 
#				Roboto-Medium.ttf COPYONLY)

#set the include directories
include_directories(
			${CMAKE_CURRENT_BINARY_DIR}
			${CMAKE_CURRENT_SOURCE_DIR}
			${GLFW_INCLUDE_DIRS}
			${TIRA_INCLUDE_DIRS}
)

#create an executable
add_executable(phys
				phys.cpp
)

target_link_libraries(phys
#				PRIVATE glfw
				PRIVATE glm::glm
#				PRIVATE GLEW::GLEW
#				${OPENGL_LIBRARIES}
#				${CMAKE_DL_LIBS}
#				PRIVATE imgui::imgui
)