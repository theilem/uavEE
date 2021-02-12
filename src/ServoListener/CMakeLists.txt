add_xplane_plugin(ServoListener 303 ServoListenerPlugin.cpp
		ServoListener.cpp
		../utils.cpp # TODO probably a better way to do that
		)

target_link_libraries(ServoListener cps_core
#		uavap_core
#		uavap_api
		Eigen3::Eigen
#		stdc++fs
		)
