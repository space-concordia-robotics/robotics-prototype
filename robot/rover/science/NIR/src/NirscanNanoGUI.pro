#-------------------------------------------------
#
# Project created by QtCreator 2014-11-07T10:24:31
#
#-------------------------------------------------

QT       += core gui
CONFIG += qwt

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
macx{
QMAKE_MAC_SDK = macosx10.11
}

TARGET = NirscanNanoGUI
TEMPLATE = app

INCLUDEPATH += $$PWD/hidapi-master/hidapi
win32: INCLUDEPATH += $$PWD/lmusb/
win32: INCLUDEPATH += $$PWD/lmdfu/
INCLUDEPATH += $$PWD/../../Common/include
INCLUDEPATH += $$PWD/Common/include
macx: INCLUDEPATH += $$PWD/../../../DLP_Spectrum_Library/src
#INCLUDEPATH += $$PWD/../../../DLP_Spectrum_Library/src
INCLUDEPATH += $$(DLPSPECLIB_INSTALL_ROOT)/src
DEPENDPATH += $$(DLPSPECLIB_INSTALL_ROOT)/src

SOURCES += main.cpp\
        mainwindow.cpp \
    usb.cpp \
    API.cpp \
    spectrum.cpp \
    scanconfigdialog.cpp \
    verticallabel.cpp \
    bluetoothdialog.cpp \
    firmware.cpp \
    scantab.cpp \
    factorytab.cpp \
    utilstab.cpp \
    testtab.cpp \
    evm.cpp \
    filepath.cpp \
    plot.cpp \
    regressiontest.cpp \
    scanconfiglist.cpp \
    versiondialog.cpp \
    filesettings_window.cpp \
    Serial.c

HEADERS  += mainwindow.h \
    API.h \
    spectrum.h \
    version.h \
    Common.h \
    scanconfigdialog.h \
    dlpspec_calib.h \
    dlpspec_scan.h \
    dlpspec_scan_col.h \
    dlpspec_util.h \
    dlpspec_scan_had.h \
    verticallabel.h \
    lmusbdll.h \
    bluetoothdialog.h \
    firmware.h \
    nnosnrdefs.h \
    scanconfiglist.h \
    evm.h \
    version.h \
    filepath.h \
    plot.h \
    refCalMatrix.h \
    regressiontest.h \
    goldensample.h \
    versiondialog.h \
    Serial.h \
    filesettings_window.h

win32: HEADERS  +=  lmdfu.h

FORMS    += mainwindow.ui \
    scanconfigdialog.ui \
    bluetoothdialog.ui \
    versiondialog.ui \
    filesettings_window.ui

RESOURCES += \
    NirscanNano.qrc


# If we have source, compile from source
exists($$(DLPSPECLIB_INSTALL_ROOT)\src\build-lib.bat) {

# Add DLP Spectrum Library as prebuild step
win32: builddlpspec.commands = $$(DLPSPECLIB_INSTALL_ROOT)\src\build-lib.bat
QMAKE_EXTRA_TARGETS += builddlpspec

# Hook our builddlpspec target in between qmake's Makefile update and the actual project target.
builddlpspechook.depends = builddlpspec
CONFIG(debug,debug|release):builddlpspechook.target = Makefile.Debug
CONFIG(release,debug|release):builddlpspechook.target = Makefile.Release
QMAKE_EXTRA_TARGETS += builddlpspechook

} else {
# Else use precompiled dlpspeclib
warning('Spectrum Library not compiled, using precompiled version if available.')
}


# Icons
macx{
ICON = icons/NIRscanNano.icns
}

win32{
RC_FILE = icons\NIRscanNano.rc
}

linux-g++{
#QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/lib
}

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/hidapi-master/windows/release/ -lhidapi
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/hidapi-master/windows/debug/ -lhidapi
macx: SOURCES += $$PWD/hidapi-master/mac/hid.c
unix: !macx: SOURCES += $$PWD/hidapi-master/linux/hid.c

macx: LIBS += -framework CoreFoundation -framework IOkit
win32: LIBS += -lSetupAPI
unix: !macx: LIBS += -lusb-1.0 -ludev

DEPENDPATH += $$PWD/hidapi-master/hidapi $$PWD/lmusb
OTHER_FILES +=  icons\NIRscanNano.rc \
                icons\NIRscanNano.ico \

macx: LIBS += -L$$PWD/../../../DLP_Spectrum_Library/src/ -lmacdlpspec
win32: LIBS += -L$$PWD/../../../DLP_Spectrum_Library/src/ -ldlpspec
win32: LIBS += -L$$(DLPSPECLIB_INSTALL_ROOT)/src/ -ldlpspec
win32: LIBS += -L$$PWD/lmusb/ -llmusbdll
win32: LIBS += -L$$PWD/lmdfu/ -llmdfu


# Deploy steps

isEmpty(TARGET_EXT) {
    win32 {
        TARGET_CUSTOM_EXT = .exe
    }
    macx {
        TARGET_CUSTOM_EXT = .app
    }
} else {
    TARGET_CUSTOM_EXT = $${TARGET_EXT}
}

win32 {
    DEPLOY_COMMAND = $$shell_quote($$shell_path(C:\Qt\5.8\mingw53_32\bin\windeployqt))
    DEPLOY_DIR = deployWindows/Binaries
    DEPLOY_HID_DLL = $$shell_quote($$shell_path($${PWD}/hidapi-master/windows/release/hidapi.dll))
}
macx {
    DEPLOY_COMMAND = macdeployqt
    DEPLOY_DIR = deployMac
}

CONFIG( debug, debug|release ) {
    # debug
    DEPLOY_TARGET = $$shell_quote($$shell_path($${OUT_PWD}/debug/$${TARGET}$${TARGET_CUSTOM_EXT}))
} else {
    # release
    DEPLOY_TARGET = $$shell_quote($$shell_path($${OUT_PWD}/release/$${TARGET}$${TARGET_CUSTOM_EXT}))
    DEPLOY_DESTINATION = $$shell_quote($$shell_path($${OUT_PWD}/$${DEPLOY_DIR}))
    DEPLOY_EXECUTABLE = $$shell_quote($$shell_path($${OUT_PWD}/$${DEPLOY_DIR}/$${TARGET}$${TARGET_CUSTOM_EXT}))

    #  # Uncomment the following line to help debug the deploy command when running qmake
      warning($${DEPLOY_COMMAND} $${DEPLOY_TARGET})

    #Remove previous deployment directory
    QMAKE_POST_LINK  = rmdir /Q /S $${DEPLOY_DESTINATION}

    #Create deployment directory
    QMAKE_POST_LINK += & mkdir $${DEPLOY_DESTINATION}

    #Copy executable into deployment directory
    QMAKE_POST_LINK += && copy $${DEPLOY_TARGET} $${DEPLOY_DESTINATION}

    #Use qt deployment for most DLLs
    QMAKE_POST_LINK += && $${DEPLOY_COMMAND} $${DEPLOY_EXECUTABLE}

    #Copy HID dll which is not a QT library
    QMAKE_POST_LINK += && copy $${DEPLOY_HID_DLL} $${DEPLOY_DESTINATION}
}
