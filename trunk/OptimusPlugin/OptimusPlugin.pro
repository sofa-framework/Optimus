load(sofa/pre)
defineAsPlugin(OptimusPlugin)

README_FILE = OptimusPlugin.txt

TARGET = OptimusPlugin 

DEFINES += SOFA_BUILD_OPTIMUSPLUGIN

INCLUDEPATH += $$SOFA_INSTALL_INC_DIR/applications/plugins
INCLUDEPATH += $$SOFA_INSTALL_INC_DIR/extlibs

HEADERS = \
initOptimusPlugin.h \
OptimParams.h \
OptimParams.inl \
ekfilter.hpp \
ekfilter_impl.inl \
kfilter.hpp \
kfilter_impl.inl \
KalmanFilter.h \
KalmanFilter.inl

SOURCES = \
initOptimusPlugin.cpp \
OptimParams.cpp \
KalmanFilter.cpp


unix : QMAKE_POST_LINK = cp $$SRC_DIR/$$README_FILE $$LIB_DESTDIR 
win32 : QMAKE_POST_LINK = copy \"$$toWindowsPath($$SRC_DIR/$$README_FILE)\" \"$$LIB_DESTDIR\"

load(sofa/post)
