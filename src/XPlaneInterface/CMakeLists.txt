include(xplane.cmake)


add_xplane_plugin(lin 303 ${CMAKE_CURRENT_SOURCE_DIR}/XPlanePlugin.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/XPlaneInterface.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/RemoteAutopilotInterface.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/DataRecording.cpp
		)


target_link_libraries(lin PUBLIC cps_core
		uavap_core
		uavap_api
		Eigen3::Eigen
		stdc++fs
		)

install(TARGETS lin
		DESTINATION lib)
