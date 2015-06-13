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

enum
  {
    EIN_EVENT_MOUSEMOVE      =0,
    EIN_EVENT_LBUTTONDOWN    =1,
    EIN_EVENT_RBUTTONDOWN    =2,
    EIN_EVENT_MBUTTONDOWN    =3,
    EIN_EVENT_LBUTTONUP      =4,
    EIN_EVENT_RBUTTONUP      =5,
    EIN_EVENT_MBUTTONUP      =6,
    EIN_EVENT_LBUTTONDBLCLK  =7,
    EIN_EVENT_RBUTTONDBLCLK  =8,
    EIN_EVENT_MBUTTONDBLCLK  =9,
    EIN_EVENT_MOUSEWHEEL     =10,
    EIN_EVENT_MOUSEHWHEEL    =11
  };


static const int tableMouseButtons[][3]={
  {EIN_EVENT_LBUTTONUP, EIN_EVENT_RBUTTONUP, EIN_EVENT_MBUTTONUP},               //mouse_up
  {EIN_EVENT_LBUTTONDOWN, EIN_EVENT_RBUTTONDOWN, EIN_EVENT_MBUTTONDOWN},         //mouse_down
  {EIN_EVENT_LBUTTONDBLCLK, EIN_EVENT_RBUTTONDBLCLK, EIN_EVENT_MBUTTONDBLCLK},   //mouse_dbclick
  {EIN_EVENT_MOUSEMOVE, EIN_EVENT_MOUSEMOVE, EIN_EVENT_MOUSEMOVE}                //mouse_move
};

typedef void (*EinMouseCallback )(int event, int x, int y, int flags, void* param);


class EinViewPort
{
public:
    virtual ~EinViewPort() {}

    virtual QWidget* getWidget() = 0;

    virtual void setMouseCallBack(EinMouseCallback m, void* param) = 0;
    virtual void writeSettings(QSettings& settings) = 0;
    virtual void readSettings(QSettings& settings) = 0;


    virtual void updateImage(const Mat arr) = 0;

    virtual void startDisplayInfo(QString text, int delayms) = 0;

    virtual void makeCurrentOpenGlContext() = 0;
    virtual void updateGl() = 0;

    virtual void setSize(QSize size_) = 0;
};



#ifdef HAVE_QT_OPENGL

class OpenGlEinViewPort : public QGLWidget, public EinViewPort
{
public:
    explicit OpenGlEinViewPort(QWidget* parent);
    ~OpenGlEinViewPort();

    QWidget* getWidget();
    void setMouseCallBack(EinMouseCallback m, void* param);
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

    
};

#endif // HAVE_QT_OPENGL


class DefaultEinViewPort : public QGraphicsView, public EinViewPort
{
    Q_OBJECT

public:
    DefaultEinViewPort(QWidget* centralWidget, int keep_ratio);
    ~DefaultEinViewPort();

    QWidget* getWidget();
    void setMouseCallBack(EinMouseCallback m, void* param);
    void writeSettings(QSettings& settings);
    void readSettings(QSettings& settings);

    void updateImage(const Mat arr);

    void startDisplayInfo(QString text, int delayms);


    void makeCurrentOpenGlContext();
    void updateGl();

    void setSize(QSize size_);
    void icvmouseHandler(QMouseEvent *event, type_mouse_event category, int &cv_event, int &flags);
    void icvmouseProcessing(QPointF pt, int cv_event, int flags);

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
    EinMouseCallback on_mouse;
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
