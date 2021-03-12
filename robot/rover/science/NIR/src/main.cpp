/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/

#include "mainwindow.h"
#include <QApplication>
#include <QSplashScreen>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QPixmap pixmap(":/new/Icons/splash.png");
    QSplashScreen splash(pixmap);
    splash.show();
    MainWindow w;
    w.show();
    splash.finish(&w);
    QFont myFont("Arial", 8, QFont::Normal);
    QApplication::setFont(myFont);

    
    return a.exec();
}
