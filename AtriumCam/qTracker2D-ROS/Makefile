#############################################################################
# Makefile for building: qTracker2D-ROS
# Generated by qmake (2.01a) (Qt 4.6.2) on: Sun Nov 27 15:18:47 2011
# Project:  qTracker2D-ROS.pro
# Template: app
# Command: /usr/bin/qmake-qt4 -spec /usr/share/qt4/mkspecs/linux-g++ -unix -o Makefile qTracker2D-ROS.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DQT_NO_DEBUG -DQT_PHONON_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED
CFLAGS        = -pipe -O2 -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -pipe -O2 -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/phonon -I/usr/include/qt4 -I/usr/local/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include -I/opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/utilities/rostime/include -I/opt/ros/electric/stacks/ros_comm/tools/rosconsole/include -I/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/include -I/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/src -I/usr/include/qt4/phonon_compat -I. -I.
LINK          = g++
LFLAGS        = -Wl,-O1
LIBS          = $(SUBLIBS)  -L/usr/lib -L/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/lib -lros -lopencv_core -lopencv_highgui -lopencv_calib3d -lopencv_contrib -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_stitching -lopencv_ts -lopencv_video -lphonon -lQtGui -lQtCore -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/bin/qmake-qt4
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = main.cpp \
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
		capturethread.cpp moc_mainwin.cpp \
		moc_trackerDialog.cpp \
		moc_perspectiveDialog.cpp \
		moc_initdialog.cpp \
		moc_calibrationDialog.cpp \
		moc_renderwidget.cpp \
		moc_controller.cpp \
		moc_processingthread.cpp
OBJECTS       = main.o \
		mainwin.o \
		trackerDialog.o \
		perspectiveDialog.o \
		initdialog.o \
		calibrationDialog.o \
		imagebuffer.o \
		filter.o \
		renderwidget.o \
		controller.o \
		processingthread.o \
		capturethread.o \
		moc_mainwin.o \
		moc_trackerDialog.o \
		moc_perspectiveDialog.o \
		moc_initdialog.o \
		moc_calibrationDialog.o \
		moc_renderwidget.o \
		moc_controller.o \
		moc_processingthread.o
DIST          = /usr/share/qt4/mkspecs/common/g++.conf \
		/usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		qTracker2D-ROS.pro
QMAKE_TARGET  = qTracker2D-ROS
DESTDIR       = 
TARGET        = qTracker2D-ROS

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET): ui_mainwin.h ui_trackerDialog.h ui_perspectiveDialog.h ui_initdialog.h ui_calibrationDialog.h $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: qTracker2D-ROS.pro  /usr/share/qt4/mkspecs/linux-g++/qmake.conf /usr/share/qt4/mkspecs/common/g++.conf \
		/usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		/usr/lib/libphonon.prl \
		/usr/lib/libQtGui.prl \
		/usr/lib/libQtCore.prl
	$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++ -unix -o Makefile qTracker2D-ROS.pro
/usr/share/qt4/mkspecs/common/g++.conf:
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/qt.prf:
/usr/share/qt4/mkspecs/features/unix/thread.prf:
/usr/share/qt4/mkspecs/features/moc.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
/usr/lib/libphonon.prl:
/usr/lib/libQtGui.prl:
/usr/lib/libQtCore.prl:
qmake:  FORCE
	@$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++ -unix -o Makefile qTracker2D-ROS.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/qTracker2D-ROS1.0.0 || $(MKDIR) .tmp/qTracker2D-ROS1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/qTracker2D-ROS1.0.0/ && $(COPY_FILE) --parents mainwin.h trackerDialog.h perspectiveDialog.h initdialog.h calibrationDialog.h imagebuffer.h filter.h renderwidget.h controller.h processingthread.h capturethread.h .tmp/qTracker2D-ROS1.0.0/ && $(COPY_FILE) --parents main.cpp mainwin.cpp trackerDialog.cpp perspectiveDialog.cpp initdialog.cpp calibrationDialog.cpp imagebuffer.cpp filter.cpp renderwidget.cpp controller.cpp processingthread.cpp capturethread.cpp .tmp/qTracker2D-ROS1.0.0/ && $(COPY_FILE) --parents mainwin.ui trackerDialog.ui perspectiveDialog.ui initdialog.ui calibrationDialog.ui .tmp/qTracker2D-ROS1.0.0/ && (cd `dirname .tmp/qTracker2D-ROS1.0.0` && $(TAR) qTracker2D-ROS1.0.0.tar qTracker2D-ROS1.0.0 && $(COMPRESS) qTracker2D-ROS1.0.0.tar) && $(MOVE) `dirname .tmp/qTracker2D-ROS1.0.0`/qTracker2D-ROS1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/qTracker2D-ROS1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all: moc_mainwin.cpp moc_trackerDialog.cpp moc_perspectiveDialog.cpp moc_initdialog.cpp moc_calibrationDialog.cpp moc_renderwidget.cpp moc_controller.cpp moc_processingthread.cpp
compiler_moc_header_clean:
	-$(DEL_FILE) moc_mainwin.cpp moc_trackerDialog.cpp moc_perspectiveDialog.cpp moc_initdialog.cpp moc_calibrationDialog.cpp moc_renderwidget.cpp moc_controller.cpp moc_processingthread.cpp
moc_mainwin.cpp: ui_mainwin.h \
		initdialog.h \
		ui_initdialog.h \
		calibrationDialog.h \
		ui_calibrationDialog.h \
		perspectiveDialog.h \
		ui_perspectiveDialog.h \
		trackerDialog.h \
		ui_trackerDialog.h \
		renderwidget.h \
		filter.h \
		controller.h \
		capturethread.h \
		mainwin.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) mainwin.h -o moc_mainwin.cpp

moc_trackerDialog.cpp: ui_trackerDialog.h \
		trackerDialog.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) trackerDialog.h -o moc_trackerDialog.cpp

moc_perspectiveDialog.cpp: ui_perspectiveDialog.h \
		perspectiveDialog.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) perspectiveDialog.h -o moc_perspectiveDialog.cpp

moc_initdialog.cpp: ui_initdialog.h \
		initdialog.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) initdialog.h -o moc_initdialog.cpp

moc_calibrationDialog.cpp: ui_calibrationDialog.h \
		calibrationDialog.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) calibrationDialog.h -o moc_calibrationDialog.cpp

moc_renderwidget.cpp: filter.h \
		renderwidget.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) renderwidget.h -o moc_renderwidget.cpp

moc_controller.cpp: capturethread.h \
		controller.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) controller.h -o moc_controller.cpp

moc_processingthread.cpp: processingthread.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) processingthread.h -o moc_processingthread.cpp

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all: ui_mainwin.h ui_trackerDialog.h ui_perspectiveDialog.h ui_initdialog.h ui_calibrationDialog.h
compiler_uic_clean:
	-$(DEL_FILE) ui_mainwin.h ui_trackerDialog.h ui_perspectiveDialog.h ui_initdialog.h ui_calibrationDialog.h
ui_mainwin.h: mainwin.ui
	/usr/bin/uic-qt4 mainwin.ui -o ui_mainwin.h

ui_trackerDialog.h: trackerDialog.ui
	/usr/bin/uic-qt4 trackerDialog.ui -o ui_trackerDialog.h

ui_perspectiveDialog.h: perspectiveDialog.ui
	/usr/bin/uic-qt4 perspectiveDialog.ui -o ui_perspectiveDialog.h

ui_initdialog.h: initdialog.ui
	/usr/bin/uic-qt4 initdialog.ui -o ui_initdialog.h

ui_calibrationDialog.h: calibrationDialog.ui
	/usr/bin/uic-qt4 calibrationDialog.ui -o ui_calibrationDialog.h

compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: compiler_moc_header_clean compiler_uic_clean 

####### Compile

main.o: main.cpp mainwin.h \
		ui_mainwin.h \
		initdialog.h \
		ui_initdialog.h \
		calibrationDialog.h \
		ui_calibrationDialog.h \
		perspectiveDialog.h \
		ui_perspectiveDialog.h \
		trackerDialog.h \
		ui_trackerDialog.h \
		renderwidget.h \
		filter.h \
		controller.h \
		capturethread.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o main.cpp

mainwin.o: mainwin.cpp mainwin.h \
		ui_mainwin.h \
		initdialog.h \
		ui_initdialog.h \
		calibrationDialog.h \
		ui_calibrationDialog.h \
		perspectiveDialog.h \
		ui_perspectiveDialog.h \
		trackerDialog.h \
		ui_trackerDialog.h \
		renderwidget.h \
		filter.h \
		controller.h \
		capturethread.h \
		processingthread.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o mainwin.o mainwin.cpp

trackerDialog.o: trackerDialog.cpp trackerDialog.h \
		ui_trackerDialog.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o trackerDialog.o trackerDialog.cpp

perspectiveDialog.o: perspectiveDialog.cpp perspectiveDialog.h \
		ui_perspectiveDialog.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o perspectiveDialog.o perspectiveDialog.cpp

initdialog.o: initdialog.cpp initdialog.h \
		ui_initdialog.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o initdialog.o initdialog.cpp

calibrationDialog.o: calibrationDialog.cpp calibrationDialog.h \
		ui_calibrationDialog.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o calibrationDialog.o calibrationDialog.cpp

imagebuffer.o: imagebuffer.cpp imagebuffer.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o imagebuffer.o imagebuffer.cpp

filter.o: filter.cpp filter.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o filter.o filter.cpp

renderwidget.o: renderwidget.cpp renderwidget.h \
		filter.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o renderwidget.o renderwidget.cpp

controller.o: controller.cpp controller.h \
		capturethread.h \
		processingthread.h \
		imagebuffer.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o controller.o controller.cpp

processingthread.o: processingthread.cpp processingthread.h \
		filter.h \
		imagebuffer.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o processingthread.o processingthread.cpp

capturethread.o: capturethread.cpp capturethread.h \
		imagebuffer.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o capturethread.o capturethread.cpp

moc_mainwin.o: moc_mainwin.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_mainwin.o moc_mainwin.cpp

moc_trackerDialog.o: moc_trackerDialog.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_trackerDialog.o moc_trackerDialog.cpp

moc_perspectiveDialog.o: moc_perspectiveDialog.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_perspectiveDialog.o moc_perspectiveDialog.cpp

moc_initdialog.o: moc_initdialog.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_initdialog.o moc_initdialog.cpp

moc_calibrationDialog.o: moc_calibrationDialog.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_calibrationDialog.o moc_calibrationDialog.cpp

moc_renderwidget.o: moc_renderwidget.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_renderwidget.o moc_renderwidget.cpp

moc_controller.o: moc_controller.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_controller.o moc_controller.cpp

moc_processingthread.o: moc_processingthread.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o moc_processingthread.o moc_processingthread.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

