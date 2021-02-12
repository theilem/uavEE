add_xplane_plugin(XPlanePlugin 303 XPlanePlugin.cpp
		XPlaneInterface.cpp
		RemoteAutopilotInterface.cpp
		DataRecording.cpp
		../utils.cpp # TODO probably a better way to do that
		)

target_link_libraries(XPlanePlugin cps_core
		uavap_core
		uavap_api
		Eigen3::Eigen
		stdc++fs
		)
