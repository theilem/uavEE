set(EIGEN_INCLUDE_DIR /usr/local/include/eigen3)
set(PROTOBUF_INCLUDE_DIR /usr/lib/protobuf)
set(UAVAP_INCLUDE_DIR /usr/local/include)
set(BOOST_INCLUDE_DIR /usr/include)

set(UAVAP_LIB_DIR /usr/local/lib)
set(BOOST_LIB_DIR /usr/local/lib)
set(PROTOBUF_LIBRARIES /usr/local/lib)

if(CMAKE_COMPILER_IS_GNUCC)
    if(CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.9.9)
        message(WARNING "gcc version too new, setting gcc-5 as C compiler")
        set(CMAKE_C_COMPILER /usr/bin/gcc-5)
    endif(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.9.9)

    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.9.9)
        message(WARNING "g++ version too new, setting g++-5 as CXX compiler")
        set(CMAKE_CXX_COMPILER /usr/bin/g++-5)
    endif(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.9.9)

    message("Using ${CMAKE_C_COMPILER} as C compiler")
    message("Using ${CMAKE_CXX_COMPILER} as CXX compiler")
    message("")
endif(CMAKE_COMPILER_IS_GNUCC)
