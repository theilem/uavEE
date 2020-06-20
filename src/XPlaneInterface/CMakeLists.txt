include(xplane.cmake)


add_xplane_plugin(lin 303 XPlanePlugin.cpp
        XPlaneNode.cpp
        )

target_link_libraries(lin cps_core
        uavap_core
        Eigen3::Eigen
        )

install(TARGETS lin
        DESTINATION lib)
