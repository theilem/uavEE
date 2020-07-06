add_library(remote_ap RemoteAutopilotInterface.cpp)

target_link_libraries(remote_ap cps_core
		uavap_core
		Eigen3::Eigen
		)

install(TARGETS lin
		DESTINATION lib)
