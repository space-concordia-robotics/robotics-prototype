
/*****************************************************************************
**
 * This module defines all the functions in the CustomPlot class
 * The CutomPlot class, provides modules to plot a particular set of x and y axis Data
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 *****************************************************************************/

#include "plot.h"
#include "math.h"


#define PLOT_HEIGHT 450
#define PLOT_WIDTH 515
#define NUM_GRID_LINES 5

CustomPlot::CustomPlot()
{
    m_prevMaxx = 0;
    m_prevMinx = 0;
    m_prevSizex = 0;
    m_cumMaxy = 0;
}
void CustomPlot::DemarcateSections(QVector<double> sectionBordersStart,QVector<double> sectionBordersEnd, int numsections)
{

    m_sectionBordersStart = sectionBordersStart;
    m_sectionBordersEnd = sectionBordersEnd;
    m_numSections = numsections;

}

double CustomPlot::ScaleToPlot(double value, double max, double widget_limit)
/**
 * This function calculates the relative position/value of
 * a point with respect to the min/max limits of the axis of plot widget
 * @param  value - I - the current value which has to scaled
 * @param  max - I - the max value to be plotted in that axis
 * @param widget_limit - I - the max height/width of the ploting widget
 * @return double - o - the scaled value
 */
{
    if(max == 0)
        return 0;
    return (value * widget_limit/ max);
}

void CustomPlot::SetUpAxes(QGraphicsScene *scene, double maxx, double maxy, double minx)
/**
 * This finction draws the axis lines and marks  grid lines
 * @param scene - I - the QGraphicScene where the axes have to be marked
 * @param maxx - I - double the maximum possible X axis value
 * @param maxy - I - the maximum possible Y axis value
 * @param minx - I - the minimum x axis value. The min y value is always zero.
 */
{
    QPen penAxis(Qt::red, 3, Qt::SolidLine, Qt::SquareCap);

    scene->addLine(0,PLOT_HEIGHT,PLOT_WIDTH,PLOT_HEIGHT, penAxis);
    scene->addLine(0,0,0,PLOT_HEIGHT, penAxis);


    int divx;

    if ((maxx-minx) > NUM_GRID_LINES)
        divx = (maxx - minx)/NUM_GRID_LINES;
    else
        divx = 1; //in case of Detector Align


    int value;

     QPen penGrid(Qt::black, 1, Qt::DotLine, Qt::SquareCap);
    //mark x axis

     QString text;

     if(minx)  //in case of Wavelengths show the minimum wavelength
     {
         int val = ceil(minx);
         text.sprintf("%d",val);
         MarkText(1,PLOT_HEIGHT - 15,scene,text);
     }
    for(int i = 1; ((i < NUM_GRID_LINES) && (divx < (maxx- minx))); i++)
    {

        value = divx * i;

        double val = ScaleToPlot(value,(maxx - minx), PLOT_WIDTH);

        int valuex = ceil(minx+value);

        text.sprintf("%d",valuex);

        scene->addLine(val,0,val,PLOT_HEIGHT,penGrid);

        MarkText(val,PLOT_HEIGHT - 15,scene,text);

    }

    //mark Y axis

    if(maxy > NUM_GRID_LINES)
    {

        int divy = maxy / NUM_GRID_LINES;

        for(int i = 1; i <NUM_GRID_LINES ; i++)
        {

           double value = divy * i;

            double val = ScaleToPlot(value,maxy, PLOT_HEIGHT - 10);

            text.sprintf("%0.2f",value);

            scene->addLine(0,PLOT_HEIGHT - 10 - val,PLOT_WIDTH,PLOT_HEIGHT - 10 - val,penGrid);

            MarkText(1,PLOT_HEIGHT - 10 - val,scene,text);
        }
    }

    else
    {

        double divy = maxy / NUM_GRID_LINES;
        double value;
        for(int i = 1; i < NUM_GRID_LINES; i++)
        {

            value  = divy * i;

            double val = ScaleToPlot(value,maxy, PLOT_HEIGHT - 10);

            text.sprintf("%0.2f",value);

            scene->addLine(0,PLOT_HEIGHT - 10 - val,PLOT_WIDTH,PLOT_HEIGHT - 10 - val,penGrid);

            MarkText(1,PLOT_HEIGHT - 10 - val,scene,text);
        }
    }

    // mark the text for the max value on both the axes

    int maxxval = ceil(maxx);

    text.sprintf("%d", maxxval);

    MarkText(PLOT_WIDTH - 50, PLOT_HEIGHT - 15, scene, text);



    text.sprintf("%0.2f", maxy);

    scene->addLine(0,0,5,0);


    MarkText(1,5, scene, text);
}

void CustomPlot::MarkText(double x, double y, QGraphicsScene *scene, QString customtext)
/**
 *This function writes the co-ordinates(the specified text) at the specified co-ordinates in the scene
 * @param x - I -  x axis position
 * @param  y - I - y axis position
 * @param scene - I - the QGraphicsScene where the text has  to be marked
 * @param customtext - I - a QString which has to be shown at the mentioned co-ordinates
 *
 */
{

    QGraphicsTextItem * io = new QGraphicsTextItem;
    io->setPos(x,y);

    QFont font = QFont(QLatin1String("sans serif"), 5 * 1.6 , QFont::Bold);

    io->setFont(font);

    if(customtext == "")
    {
        QString text;

        text.sprintf("%0.1f,%0.1f", x, y);

        io->setPlainText(text);
    }
    else
        io->setPlainText(customtext);

    scene->addItem(io);
}

bool CustomPlot::Plot(QVector<double> xData, QVector<double>  yData ,double maxY , QGraphicsScene * scene)
/**
 * This function plots the xData on x-axis co-ordinates and yData on y-axis co-ordinates
 * in the given GraphicScene and joins adjacent points by a line
 * @param xData - I - the x-axis co-ordinates as a vector of double
 * @param yData - I - the y-axis co-ordinates as a vector of double
 * @param maxY - I - the max possible Y value, needed in case of overlaying the plots
 * @param scene - I - the QGraphicsScene where the points have to plotted
 *
 */
{
    double x1, x2;
    double y1, y2;

    bool plot_valid = true;

    if((xData.size() <= 0) || (yData.size() <= 0))
        return false;
    if(xData.size() != yData.size())
        return false;

    double maxx = xData[xData.size()-1];
    double minx =  xData[0];

    QPen pensection1(Qt::blue, 1, Qt::SolidLine, Qt::SquareCap);
    QPen pensection2(Qt::green, 1, Qt::SolidLine, Qt::SquareCap);
    QPen pensection3(Qt::magenta, 1, Qt::SolidLine, Qt::SquareCap);
    QPen pensection4(Qt::darkCyan, 1, Qt::SolidLine, Qt::SquareCap);
    QPen pensection5(Qt::red, 1, Qt::SolidLine, Qt::SquareCap);
    QVector<QPen> PenVector;
    PenVector.clear();
    PenVector.push_back(pensection1);
    PenVector.push_back(pensection2);
    PenVector.push_back(pensection3);
    PenVector.push_back(pensection4);
    PenVector.push_back(pensection5);
    if((maxx != m_prevMaxx) || (minx != m_prevMinx) || (xData.size() != m_prevSizex))
    {
        m_yValues.clear();
        m_prevMaxx = maxx;
        m_prevMinx = minx;
        m_prevSizex = xData.size();
        m_cumMaxy = maxY;
    }
    else if(maxY > m_cumMaxy)
        m_cumMaxy = maxY;

    if(minx > maxx)//in case of full dmd scan
    {
        //exchange min and max wavelengths
        double temp;
        temp = minx;
        minx = maxx;
        maxx = temp;
    }

    double xRange = maxx - minx;
    m_yValues.push_back(yData);

    SetUpAxes(scene, maxx, m_cumMaxy, minx);
    double minx_plot = ScaleToPlot(minx, xRange,PLOT_WIDTH);

    QPen pen1(Qt::blue, 0, Qt::SolidLine, Qt::SquareCap);

    QPen pen_nan(Qt::magenta, 5, Qt::SolidLine, Qt::SquareCap);


    int section;
    for(int j = 0; j < m_yValues.size(); j++)
    {
        section = 0;

        for(int i=0; i< xData.size() - 1; i++)
        {
            x1 = xData[i];

            if((m_numSections > section) && (x1 >= m_sectionBordersStart[section]))
            {
                pen1 = PenVector[section];

            }
            //do not connect different sections
            //visually showing the discontinuities in sections
            if(x1 > m_sectionBordersEnd[section])
            {
               section++;
               continue;
            }
            x1 = ScaleToPlot(x1, xRange,PLOT_WIDTH) - minx_plot;
            x2 = xData[i+1];


            x2 = ScaleToPlot(x2, xRange,PLOT_WIDTH)- minx_plot;

            y1 = m_yValues[j][i];

            if((std::isnan(y1) == true) || (std::isinf(y1) == true))
            {
                scene->addLine(x1,0,x1,PLOT_HEIGHT,pen_nan);
                plot_valid = false;
                continue;
            }

            y1 = ScaleToPlot(y1, m_cumMaxy,PLOT_HEIGHT);
            y1 = PLOT_HEIGHT - y1 + 10; //Y Axis goes down

            y2 = m_yValues[j][i+1];
            y2 = ScaleToPlot(y2, m_cumMaxy,PLOT_HEIGHT);
            y2 = PLOT_HEIGHT- y2 + 10; //leaving 10 for making the sections

            scene->addLine(x1,y1,x2,y2,pen1);
        }
    }

    return plot_valid;

}

void CustomPlot::ResetValues()
/**
 * Clears the current vector of Y axis values stored in case of overlaying plots
 *
 */
{
     m_yValues.clear();
     m_prevMaxx = 0;
     m_prevMinx = 0;
     m_prevSizex = 0;
     m_cumMaxy = 0;
}
