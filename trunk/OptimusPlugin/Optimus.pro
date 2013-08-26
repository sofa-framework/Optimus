load(sofa/pre)
defineAsPlugin(Optimus)

README_FILE = Optimus.txt

TARGET = Optimus 

DEFINES += SOFA_BUILD_OPTIMUS

INCLUDEPATH += $$SOFA_INSTALL_INC_DIR/applications/plugins
INCLUDEPATH += $$SOFA_INSTALL_INC_DIR/extlibs

HEADERS = \
	src/initOptimusPlugin.h \
	src/OptimParams.h \
	src/OptimParams.inl \
	src/ekfilter.hpp \
	src/ekfilter_impl.inl \
	src/kfilter.hpp \
	src/kfilter_impl.inl \
	src/KalmanFilter.h \
	src/KalmanFilter.inl \
 	src/TestingParams.h \
	src/BubblePackingForceField.h \
	src/BubblePackingForceField.inl

SOURCES = \
	src/initOptimusPlugin.cpp \
	src/OptimParams.cpp \
	src/KalmanFilter.cpp \
	src/TestingParams.cpp \
	src/BubblePackingForceField.cpp

unix : QMAKE_POST_LINK = cp $$SRC_DIR/$$README_FILE $$LIB_DESTDIR 
win32 : QMAKE_POST_LINK = copy \"$$toWindowsPath($$SRC_DIR/$$README_FILE)\" \"$$LIB_DESTDIR\"

load(sofa/post)
