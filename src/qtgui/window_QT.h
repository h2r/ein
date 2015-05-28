//IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

// By downloading, copying, installing or using the software you agree to this license.
// If you do not agree to this license, do not download, install,
// copy or use the software.


//                          License Agreement
//               For Open Source Computer Vision Library

//Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
//Copyright (C) 2008-2010, Willow Garage Inc., all rights reserved.
//Third party copyrights are property of their respective owners.

//Redistribution and use in source and binary forms, with or without modification,
//are permitted provided that the following conditions are met:

//  * Redistribution's of source code must retain the above copyright notice,
//  this list of conditions and the following disclaimer.

//  * Redistribution's in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.

//  * The name of the copyright holders may not be used to endorse or promote products
//  derived from this software without specific prior written permission.

//This software is provided by the copyright holders and contributors "as is" and
//any express or implied warranties, including, but not limited to, the implied
//warranties of merchantability and fitness for a particular purpose are disclaimed.
//In no event shall the Intel Corporation or contributors be liable for any direct,
//indirect, incidental, special, exemplary, or consequential damages
//(including, but not limited to, procurement of substitute goods or services;
//loss of use, data, or profits; or business interruption) however caused
//and on any theory of liability, whether in contract, strict liability,
//or tort (including negligence or otherwise) arising in any way out of
//the use of this software, even if advised of the possibility of such damage.

//--------------------Google Code 2010 -- Yannick Verdie--------------------//
#ifndef __OPENCV_HIGHGUI_QT_H__
#define __OPENCV_HIGHGUI_QT_H__

#if defined( HAVE_QT_OPENGL )
#include <QtOpenGL>
#include <QGLWidget>
#endif

#include <QAbstractEventDispatcher>
#include <QApplication>
#include <QFile>
#include <QPushButton>
#include <QGraphicsView>
#include <QSizePolicy>
#include <QInputDialog>
#include <QBoxLayout>
#include <QSettings>
#include <qtimer.h>
#include <QWaitCondition>
#include <QKeyEvent>
#include <QMetaObject>
#include <QPointer>
#include <QSlider>
#include <QLabel>
#include <QIODevice>
#include <QShortcut>
#include <QStatusBar>
#include <QVarLengthArray>
#include <QFileInfo>
#include <QDate>
#include <QFileDialog>
#include <QToolBar>
#include <QAction>
#include <QPushButton>
#include <QCheckBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QMenu>

#include <cv.h>

using namespace cv;
//start private enum
enum { CV_MODE_NORMAL = 0, CV_MODE_OPENGL = 1 };

enum type_mouse_event { mouse_up = 0, mouse_down = 1, mouse_dbclick = 2, mouse_move = 3 };

/* static const int tableMouseButtons[][3]={ */
/*   {CV_EVENT_LBUTTONUP, CV_EVENT_RBUTTONUP, CV_EVENT_MBUTTONUP},                */
/*   //mouse_up */
/*   {CV_EVENT_LBUTTONDOWN, CV_EVENT_RBUTTONDOWN, CV_EVENT_MBUTTONDOWN},          */
/*   //mouse_down */
/*   {CV_EVENT_LBUTTONDBLCLK, CV_EVENT_RBUTTONDBLCLK, CV_EVENT_MBUTTONDBLCLK},    */
/*   //mouse_dbclick */
/*   {CV_EVENT_MOUSEMOVE, CV_EVENT_MOUSEMOVE, CV_EVENT_MOUSEMOVE}                 */
/*   //mouse_move */


class ViewPort
{
public:
    virtual ~ViewPort() {}

    virtual QWidget* getWidget() = 0;


    virtual void writeSettings(QSettings& settings) = 0;
    virtual void readSettings(QSettings& settings) = 0;


    virtual void updateImage(const Mat arr) = 0;

    virtual void startDisplayInfo(QString text, int delayms) = 0;

    virtual void makeCurrentOpenGlContext() = 0;
    virtual void updateGl() = 0;

    virtual void setSize(QSize size_) = 0;
};



#ifdef HAVE_QT_OPENGL

class OpenGlViewPort : public QGLWidget, public ViewPort
{
public:
    explicit OpenGlViewPort(QWidget* parent);
    ~OpenGlViewPort();

    QWidget* getWidget();

    void writeSettings(QSettings& settings);
    void readSettings(QSettings& settings);

    void updateImage(const Mat arr);

    void startDisplayInfo(QString text, int delayms);

    void makeCurrentOpenGlContext();
    void updateGl();

    void setSize(QSize size_);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseDoubleClickEvent(QMouseEvent* event);

    QSize sizeHint() const;

private:
    QSize size;

    void icvmouseProcessing(QPointF pt, int cv_event, int flags);
};

#endif // HAVE_QT_OPENGL


class DefaultViewPort : public QGraphicsView, public ViewPort
{
    Q_OBJECT

public:
    DefaultViewPort(QWidget* centralWidget, int keep_ratio);
    ~DefaultViewPort();

    QWidget* getWidget();

    void writeSettings(QSettings& settings);
    void readSettings(QSettings& settings);

    void updateImage(const Mat arr);

    void startDisplayInfo(QString text, int delayms);


    void makeCurrentOpenGlContext();
    void updateGl();

    void setSize(QSize size_);

public slots:
    //reference:
    //http://www.qtcentre.org/wiki/index.php?title=QGraphicsView:_Smooth_Panning_and_Zooming
    //http://doc.qt.nokia.com/4.6/gestures-imagegestures-imagewidget-cpp.html

    void siftWindowOnLeft();
    void siftWindowOnRight();
    void siftWindowOnUp() ;
    void siftWindowOnDown();

    void resetZoom();
    void imgRegion();
    void ZoomIn();
    void ZoomOut();

    void saveView();

protected:
    void resizeEvent(QResizeEvent* event);
    void paintEvent(QPaintEvent* paintEventInfo);
    void wheelEvent(QWheelEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseDoubleClickEvent(QMouseEvent* event);

private:
    int param_keepRatio;

    //parameters (will be save/load)
    QTransform param_matrixWorld;

    CvMat* image2Draw_mat;
    QImage image2Draw_qt;
    int nbChannelOriginImage;

    //for mouse callback
    void* on_mouse_param;


    void scaleView(qreal scaleFactor, QPointF center);
    void moveView(QPointF delta);

    QPoint mouseCoordinate;
    QPointF positionGrabbing;
    QRect  positionCorners;
    QTransform matrixWorld_inv;
    float ratioX, ratioY;

    bool isSameSize(IplImage* img1,IplImage* img2);

    QSize sizeHint() const;
    QPointer<QWidget> centralWidget;
    QPointer<QTimer> timerDisplay;
    bool drawInfo;
    QString infoText;
    QRectF target;

    void drawInstructions(QPainter *painter);
    void drawViewOverview(QPainter *painter);
    void drawImgRegion(QPainter *painter);
    void draw2D(QPainter *painter);
    void drawStatusBar();
    void controlImagePosition();

private slots:
    void stopDisplayInfo();
};

#endif
