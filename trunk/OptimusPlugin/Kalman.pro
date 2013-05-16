######  PLUGIN TARGET
load(sofa/pre)
defineAsPlugin(Kalman)

TARGET = Kalman

DEFINES += SOFA_BUILD_KALMANPLUGIN

SOURCES = \
initKalmanPlugin.cpp \
#OptimParams.cpp \
KalmanFilter.cpp

HEADERS = \
initKalmanPlugin.h \
#OptimParams.h \
#OptimParams.inl \
ekfilter.hpp \
ekfilter_impl.inl \
kfilter.hpp \
kfilter_impl.inl \
KalmanFilter.h \
KalmanFilter.inl

README_FILE = KalmanFilter.txt

unix {
LIBS +=
}

win32 {
LIBS +=
}

unix : QMAKE_POST_LINK = cp $$SRC_DIR/$$README_FILE $$LIB_DESTDIR 
win32 : QMAKE_POST_LINK = copy \"$$toWindowsPath($$SRC_DIR/$$README_FILE)\" \"$$LIB_DESTDIR\"

load(sofa/post)
