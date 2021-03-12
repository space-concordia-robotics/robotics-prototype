/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/
#include "verticallabel.h"

VertLabel::VertLabel(QWidget *parent): QLabel(parent)
{

}

VertLabel::VertLabel(QString txt, QWidget *parent): QLabel(parent)
{
    text = txt;
}

void VertLabel::setText(const QString txt)
{
    text = txt;
}

void VertLabel::paintEvent(QPaintEvent *)
{
        QPainter painter(this);
        painter.translate( sizeHint().width(), sizeHint().height() );
        painter.rotate(270);
        painter.drawText(QPoint(0,-25), text);

}
