QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14

INCLUDEPATH += include
INCLUDEPATH += /opt/ros/foxy/include

SOURCES += \
    src/main.cpp \
    src/ros/ros_node.cpp \
    src/ui/mainwindow.cpp

HEADERS += \
    include/qt_turtle/ros/ros_node.h \
    include/qt_turtle/ui/mainwindow.h

FORMS += \
    ui/mainwindow.ui

DISTFILES += \
      package.xml \
      CMakeLists.txt \
      launch/qt_turtle.launch.py
