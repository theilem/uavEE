# Created by and for Qt Creator This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

#TARGET = uavEE

HEADERS += $$system(find -name "*.h")
HEADERS += $$system(find -name "*.hpp")
HEADERS += $$system(find -name "CMakeLists.txt")

SOURCES += $$system(find -name "*.c")
SOURCES += $$system(find -name "*.cpp")
SOURCES += $$system(find -name "CMakeLists.txt")

OTHER_FILES += $$system(find -type f)

INCLUDEPATH = \
	$$PWD/autopilot_interface/include/ \
	$$PWD/ground_station/include/ \
	$$PWD/pan_tilt_handler/include/ \
	$$PWD/power_modeling/include/ \
	$$PWD/radio_comm/include/ \
	$$PWD/simulation_interface/include \
	$$PWD/x_plane_interface/include/

#DEFINES = 
