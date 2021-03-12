/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/
#ifndef VERTICALLABEL_H
#define VERTICALLABEL_H


#include <QLabel>
#include <QPaintEvent>
#include <QPainter>

class VertLabel : public QLabel
{
    Q_OBJECT
public:
    explicit VertLabel(QString txt , QWidget *parent = 0);
    explicit VertLabel(QWidget *parent = 0);

signals:

protected:
    void paintEvent(QPaintEvent *);


public slots:
     void setText(const QString txt);

private:
    QString text;

};




#endif // VERTICALLABEL_H

