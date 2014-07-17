TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += \
    src/save_map_msg.cpp \
    src/extracting.cpp \
    src/pcl_view.cpp \
    src/linear_segmentation.cpp \
    src/save_map_vector.cpp \
    src/line.cpp \
    src/test.cpp \
    src/wall.cpp

OTHER_FILES += \
    CMakeLists.txt

INCLUDEPATH += /usr/include/pcl-1.7 \
        /opt/ros/indigo/include

HEADERS += \
    src/includes.h \
    src/line.h \
    src/linear_segmentation.h \
    src/pcl_view.h \
    src/wall.h
