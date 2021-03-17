include(xplane.cmake)


add_xplane_plugin(lin 303 XPlanePlugin.cpp
		XPlaneInterface.cpp
		RemoteAutopilotInterface.cpp
		DataRecording.cpp
		)

target_link_libraries(lin cps_core
		uavap_core
		uavap_api
		Eigen3::Eigen
		stdc++fs
		)

install(TARGETS lin
		DESTINATION lib)
