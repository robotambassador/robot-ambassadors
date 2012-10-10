# -------------------------------------------------
# Project created by QtCreator 2010-07-12T20:25:59
# -------------------------------------------------
QT += phonon
TARGET = qTracker2D-ROS
TEMPLATE = app
LIBS += -L/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib -lros \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_calib3d \
    -lopencv_contrib \
    -lopencv_features2d \
    -lopencv_flann \
    -lopencv_gpu \
    -lopencv_imgproc \
    -lopencv_legacy \
    -lopencv_ml \
    -lopencv_objdetect \
    -lopencv_stitching \
    -lopencv_ts \
    -lopencv_video

INCLUDEPATH += "/usr/local/include" \
    "/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include" \
    "/opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/cpp/include" \
    "/opt/ros/electric/stacks/ros_comm/utilities/rostime/include" \
    "/opt/ros/electric/stacks/ros_comm/tools/rosconsole/include" \
    "/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/include" \
    "/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/include" \
    "/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/include" \
    "/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/src"

SOURCES += main.cpp \
    mainwin.cpp \
    trackerDialog.cpp \
    perspectiveDialog.cpp \
    initdialog.cpp \
    calibrationDialog.cpp \
    imagebuffer.cpp \
    filter.cpp \
    renderwidget.cpp \
    controller.cpp \
    processingthread.cpp \
    capturethread.cpp
HEADERS += mainwin.h \
    trackerDialog.h \
    perspectiveDialog.h \
    initdialog.h \
    calibrationDialog.h \
    imagebuffer.h \
    filter.h \
    renderwidget.h \
    controller.h \
    processingthread.h \
    capturethread.h
FORMS += mainwin.ui \
    trackerDialog.ui \
    perspectiveDialog.ui \
    initdialog.ui \
    calibrationDialog.ui
