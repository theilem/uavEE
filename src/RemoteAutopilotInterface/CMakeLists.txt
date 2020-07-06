add_library(remote_ap RemoteAutopilotInterface.cpp)

target_link_libraries(remote_ap cps_core
		uavap_core
		)

install(TARGETS lin
		DESTINATION lib)
