QT       += core gui sql network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS CT_USE_LAPACK

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    drawing.cpp \
    EigenQP.cpp \
    jsonxx/jsonxx.cc \
    lqrhandler.cpp \
    main.cpp \
    gsmainwindow.cpp \
    mosaicprocessor.cpp \
    positiondata.cpp \
    ROV.cpp \
    spacemousecontroller.cpp \
    src/Rov_Tcp_Client_Qt/Ahrs/ahrsconfigure.cpp \
    src/Rov_Tcp_Client_Qt/Ahrs/ahrsreaddata.cpp \
    src/Rov_Tcp_Client_Qt/Motor_Control/motor_control.cpp \
    src/Rov_Tcp_Client_Qt/Pressure/pressure.cpp \
    src/Rov_Tcp_Client_Qt/Tcp_Connection/tcpclientsocket.cpp \
    src/Rov_Tcp_Client_Qt/gupikmodules.cpp \
    src/Rov_Tcp_Client_Qt/lykacz/lykacz.cpp \
    tcpconnectionhandler.cpp \
    typedefs.cpp \
    videoprocess.cpp

HEADERS += \
    drawing.h \
    EigenQP.h \
    gsmainwindow.h \
    jsonxx/jsonxx.h \
    lqrhandler.h \
    mosaicprocessor.h \
    positiondata.h \
    ROV.h \
    spacemousecontroller.h \
    src/Rov_Tcp_Client_Qt/Ahrs/ahrsconfigure.h \
    src/Rov_Tcp_Client_Qt/Ahrs/ahrsreaddata.h \
    src/Rov_Tcp_Client_Qt/Motor_Control/motor_control.h \
    src/Rov_Tcp_Client_Qt/Pressure/pressure.h \
    src/Rov_Tcp_Client_Qt/Tcp_Connection/tcpclientsocket.h \
    src/Rov_Tcp_Client_Qt/gupikmodules.h \
    src/Rov_Tcp_Client_Qt/lykacz/lykacz.h \
    tcpconnectionhandler.h \
    typedefs.h \
    videoprocess.h

FORMS += \
    gsmainwindow.ui \

TRANSLATIONS += \
    GroundStation_en_US.ts

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
INCLUDEPATH += /usr/local/include/opencv4/ /usr/include/libevdev-1.0/ /opt/ros/noetic/include/ /opt/ros/noetic/lib/
LIBS += -L/usr/local/lib -lopencv_highgui -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_videoio -lopencv_highgui -lopencv_xfeatures2d -lopencv_features2d -lopencv_calib3d -levdev -llapack -lblas -lcpp_common -lxmlrpcpp -lcv_bridge

DISTFILES += \ \
    .clang-format

RESOURCES += \
    resources.qrc
