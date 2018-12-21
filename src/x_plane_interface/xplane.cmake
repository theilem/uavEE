# Set up the imported targets for X-Plane plugin libraries:

IF(APPLE)
    SET(XPLANE_LIBRARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk/Libraries/Mac/")
ELSEIF(WIN32)
    SET(XPLANE_LIBRARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk/Libraries/Win/")
ENDIF()

SET(XPLANE_INCLUDES_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/xPlane/CHeaders/")



FUNCTION(add_xplane_sdk_definitions library_name library_version) 
    IF(APPLE)
        TARGET_COMPILE_DEFINITIONS(${library_name} PUBLIC -DAPL=1 -DIBM=0 -DLIN=0)
    ELSEIF(UNIX)
        TARGET_COMPILE_DEFINITIONS(${library_name} PUBLIC -DAPL=0 -DIBM=0 -DLIN=1)
    ELSEIF(WIN32)
        TARGET_COMPILE_OPTIONS(${library_name} PRIVATE "/MD$<$<CONFIG:Debug>:d>")
        TARGET_COMPILE_DEFINITIONS(${library_name} PUBLIC -DAPL=0 -DIBM=1 -DLIN=0)
    ENDIF()

    IF(library_version EQUAL 2)
        TARGET_COMPILE_DEFINITIONS(${library_name} PUBLIC -DXPLM200=1 -DXPLM210=1)
    ELSEIF(library_version EQUAL 3)
        TARGET_COMPILE_DEFINITIONS(${library_name} PUBLIC -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1)
    ELSE()
        MESSAGE(FATAL_ERROR "Library version must be 2 or 3")
    ENDIF()
    TARGET_INCLUDE_DIRECTORIES(${library_name} PUBLIC "${XPLANE_INCLUDES_PATH}/XPLM" "${XPLANE_INCLUDES_PATH}/Widgets" "${XPLANE_INCLUDES_PATH}/Wrappers")
ENDFUNCTION(add_xplane_sdk_definitions)

# Set up an X-Plane plugin
FUNCTION(add_xplane_plugin library_name library_version ...)
    SET(FILES ${ARGV})
    LIST(REMOVE_AT FILES 0)
    LIST(REMOVE_AT FILES 0)
    MESSAGE(STATUS "Making X-Plane plugin called ${library_name}")

    IF(APPLE)
        ADD_EXECUTABLE(${library_name} ${FILES})
        SET_TARGET_PROPERTIES(${library_name} PROPERTIES OUTPUT_NAME "mac.xpl")
    ELSEIF(UNIX)
        ADD_LIBRARY(${library_name} SHARED ${FILES})
        SET_TARGET_PROPERTIES(${library_name} PROPERTIES OUTPUT_NAME "lin")
	SET_TARGET_PROPERTIES(${library_name} PROPERTIES SUFFIX ".xpl")
	SET_TARGET_PROPERTIES(${library_name} PROPERTIES PREFIX "")
    ELSEIF(WIN32)
        ADD_LIBRARY(${library_name} MODULE ${FILES})
        TARGET_COMPILE_DEFINITIONS(${library_name} PUBLIC -D_CRT_SECURE_NO_WARNINGS /bigobj)
        SET_TARGET_PROPERTIES(${library_name} PROPERTIES OUTPUT_NAME "win.xpl")
    ENDIF()
    add_xplane_sdk_definitions(${library_name} ${library_version})

    #link flags
    IF(APPLE)
        SET_PROPERTY(TARGET ${library_name} APPEND_STRING PROPERTY LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fPIC -fvisibility=hidden -bundle")
    ELSEIF(UNIX)
        SET_PROPERTY(TARGET ${library_name} APPEND_STRING PROPERTY LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs -undefined_warning -fPIC -fvisibility=hidden")
    ENDIF()

	#TARGET_LINK_LIBRARIES(${library_name} PUBLIC ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
    #TARGET_INCLUDE_DIRECTORIES(${library_name} PUBLIC "${XPLANE_INCLUDES_PATH}/XPLM" "${XPLANE_INCLUDES_PATH}/Widgets" "${XPLANE_INCLUDES_PATH}/Wrappers")
ENDFUNCTION(add_xplane_plugin)
