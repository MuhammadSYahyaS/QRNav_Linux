TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

HEADERS += \
            withrobot_camera.hpp \
            withrobot_utility.hpp \
            tldpSerial.hpp \
            withrobot_debug_print.h

SOURCES += \
            withrobot_camera.cpp \
            withrobot_utility.cpp \
            tldpSerial.cpp \
            QRNav.cpp

LIBS += \
    -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_videoio \
    -L/usr/lib -lv4l2 -ludev \
    -L/usr/lib/x86_64-linux-gnu -lzbar -lpthread
