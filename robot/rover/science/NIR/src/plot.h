/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/
#ifndef MYPLOT_H
#define MYPLOT_H

#endif // MYPLOT_H

#include <QVector>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsTextItem>
#include <stdio.h>


class CustomPlot
{

public:
    CustomPlot();
    void MarkText(double x, double y, QGraphicsScene *scene, QString customtext = "");
    void SetUpAxes(QGraphicsScene *scene, double maxx, double maxy, double minx = 0);
    double ScaleToPlot(double value, double max, double widget_limit);
    bool  Plot(QVector<double> xData, QVector<double>  yData ,double maxY , QGraphicsScene * scene);
    void ResetValues();
    void DemarcateSections(QVector<double> sectionBordersStart,QVector<double> sectionBordersEnd, int numsections);

    //CR18634 - KV
    double m_prevMaxx;
    double m_prevMinx;
    int m_prevSizex;
    double m_cumMaxy;
    QVector<double> m_sectionBordersStart;
    QVector<double> m_sectionBordersEnd;
    int m_numSections;

    QVector < QVector<double> > m_yValues;
};
