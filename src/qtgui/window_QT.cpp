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





#include <memory>
#include <iostream>
using namespace std;

#include "window_QT.h"

#include <math.h>

#include <unistd.h>

#include <opencv2/highgui/highgui_c.h>

#ifdef HAVE_QT_OPENGL
    #ifdef Q_WS_X11
        #include <GL/glx.h>
    #endif
#endif


static const unsigned int threshold_zoom_img_region = 30;



//////////////////////////////////////////////////////
// DefaultEinViewPort

DefaultEinViewPort::DefaultEinViewPort(QWidget* arg, int keep_ratio) : QGraphicsView(arg)
{

    image2Draw_mat = 0;
    centralWidget = arg;
    param_keepRatio = keep_ratio;

    setContentsMargins(0, 0, 0, 0);
    setMinimumSize(1, 1);
    setAlignment(Qt::AlignHCenter);

    setObjectName(QString::fromUtf8("graphicsView"));

    timerDisplay = new QTimer(this);
    timerDisplay->setSingleShot(true);
    connect(timerDisplay, SIGNAL(timeout()), this, SLOT(stopDisplayInfo()));

    drawInfo = false;
    positionGrabbing = QPointF(0, 0);
    positionCorners = QRect(0, 0, size().width(), size().height());

    on_mouse = 0;
    on_mouse_param = 0;
    mouseCoordinate = QPoint(-1, -1);

    //no border
    setStyleSheet( "QGraphicsView { border-style: none; }" );

    image2Draw_mat = cvCreateMat(viewport()->height(), viewport()->width(), CV_8UC3);
    cvZero(image2Draw_mat);

    nbChannelOriginImage = 0;

    setInteractive(false);
    setMouseTracking(true); //receive mouse event everytime
}


DefaultEinViewPort::~DefaultEinViewPort()
{
    if (image2Draw_mat)
        cvReleaseMat(&image2Draw_mat);
}


QWidget* DefaultEinViewPort::getWidget()
{
    return this;
}


void DefaultEinViewPort::setMouseCallBack(EinMouseCallback m, void* param)
{
  on_mouse = m;
  on_mouse_param = param;
}



void DefaultEinViewPort::writeSettings(QSettings& settings)
{
    settings.setValue("matrix_view.m11", param_matrixWorld.m11());
    settings.setValue("matrix_view.m12", param_matrixWorld.m12());
    settings.setValue("matrix_view.m13", param_matrixWorld.m13());
    settings.setValue("matrix_view.m21", param_matrixWorld.m21());
    settings.setValue("matrix_view.m22", param_matrixWorld.m22());
    settings.setValue("matrix_view.m23", param_matrixWorld.m23());
    settings.setValue("matrix_view.m31", param_matrixWorld.m31());
    settings.setValue("matrix_view.m32", param_matrixWorld.m32());
    settings.setValue("matrix_view.m33", param_matrixWorld.m33());
}


void DefaultEinViewPort::readSettings(QSettings& settings)
{
    qreal m11 = settings.value("matrix_view.m11", param_matrixWorld.m11()).toDouble();
    qreal m12 = settings.value("matrix_view.m12", param_matrixWorld.m12()).toDouble();
    qreal m13 = settings.value("matrix_view.m13", param_matrixWorld.m13()).toDouble();
    qreal m21 = settings.value("matrix_view.m21", param_matrixWorld.m21()).toDouble();
    qreal m22 = settings.value("matrix_view.m22", param_matrixWorld.m22()).toDouble();
    qreal m23 = settings.value("matrix_view.m23", param_matrixWorld.m23()).toDouble();
    qreal m31 = settings.value("matrix_view.m31", param_matrixWorld.m31()).toDouble();
    qreal m32 = settings.value("matrix_view.m32", param_matrixWorld.m32()).toDouble();
    qreal m33 = settings.value("matrix_view.m33", param_matrixWorld.m33()).toDouble();

    param_matrixWorld = QTransform(m11, m12, m13, m21, m22, m23, m31, m32, m33);
}


void DefaultEinViewPort::updateImage(const Mat arr)
{
  CvMat cvimg = arr;
  CV_Assert(&cvimg);

  CvMat* mat = & cvimg;
  
  
  if (!image2Draw_mat || !CV_ARE_SIZES_EQ(image2Draw_mat, mat))
    {
      if (image2Draw_mat) {
        cvReleaseMat(&image2Draw_mat);
      }
      
      //the image in ipl (to do a deep copy with cvCvtColor)
      image2Draw_mat = cvCreateMat(mat->rows, mat->cols, CV_8UC3);
      image2Draw_qt = QImage(image2Draw_mat->data.ptr, image2Draw_mat->cols, image2Draw_mat->rows, image2Draw_mat->step, QImage::Format_RGB888);
      setMaximumSize(image2Draw_qt.width(), image2Draw_qt.height());
      setMinimumSize(image2Draw_qt.width(), image2Draw_qt.height());
      
      //use to compute mouse coordinate, I need to update the ratio here and in resizeEvent
      ratioX = width() / float(image2Draw_mat->cols);
      ratioY = height() / float(image2Draw_mat->rows);
      updateGeometry();
    }
  
  nbChannelOriginImage = cvGetElemType(mat);
  
  cvConvertImage(mat, image2Draw_mat, CV_CVTIMG_SWAP_RB);
  
  viewport()->update();
}


void DefaultEinViewPort::startDisplayInfo(QString text, int delayms)
{
    if (timerDisplay->isActive())
        stopDisplayInfo();

    infoText = text;
    if (delayms > 0) timerDisplay->start(delayms);
    drawInfo = true;
}



void DefaultEinViewPort::makeCurrentOpenGlContext()
{
    CV_Error(CV_OpenGlNotSupported, "Window doesn't support OpenGL");
}


void DefaultEinViewPort::updateGl()
{
    CV_Error(CV_OpenGlNotSupported, "Window doesn't support OpenGL");
}


//Note: move 2 percent of the window
void DefaultEinViewPort::siftWindowOnLeft()
{
    float delta = 2 * width() / (100.0 * param_matrixWorld.m11());
    moveView(QPointF(delta, 0));
}


//Note: move 2 percent of the window
void DefaultEinViewPort::siftWindowOnRight()
{
    float delta = -2 * width() / (100.0 * param_matrixWorld.m11());
    moveView(QPointF(delta, 0));
}


//Note: move 2 percent of the window
void DefaultEinViewPort::siftWindowOnUp()
{
    float delta = 2 * height() / (100.0 * param_matrixWorld.m11());
    moveView(QPointF(0, delta));
}


//Note: move 2 percent of the window
void DefaultEinViewPort::siftWindowOnDown()
{
    float delta = -2 * height() / (100.0 * param_matrixWorld.m11());
    moveView(QPointF(0, delta));
}


void DefaultEinViewPort::imgRegion()
{
    scaleView((threshold_zoom_img_region / param_matrixWorld.m11() - 1) * 5, QPointF(size().width() / 2, size().height() / 2));
}


void DefaultEinViewPort::resetZoom()
{
    param_matrixWorld.reset();
    controlImagePosition();
}


void DefaultEinViewPort::ZoomIn()
{
    scaleView(0.5, QPointF(size().width() / 2, size().height() / 2));
}


void DefaultEinViewPort::ZoomOut()
{
    scaleView(-0.5, QPointF(size().width() / 2, size().height() / 2));
}


//can save as JPG, JPEG, BMP, PNG
void DefaultEinViewPort::saveView()
{
    QDate date_d = QDate::currentDate();
    QString date_s = date_d.toString("dd.MM.yyyy");
    QString name_s = centralWidget->windowTitle() + "_screenshot_" + date_s;

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File %1").arg(name_s), name_s + ".png", tr("Images (*.png *.jpg *.bmp *.jpeg)"));

    if (!fileName.isEmpty()) //save the picture
    {
        QString extension = fileName.right(3);

        // Create a new pixmap to render the viewport into
        QPixmap viewportPixmap(viewport()->size());
        viewport()->render(&viewportPixmap);

        // Save it..
        if (QString::compare(extension, "png", Qt::CaseInsensitive) == 0)
        {
            viewportPixmap.save(fileName, "PNG");
            return;
        }

        if (QString::compare(extension, "jpg", Qt::CaseInsensitive) == 0)
        {
            viewportPixmap.save(fileName, "JPG");
            return;
        }

        if (QString::compare(extension, "bmp", Qt::CaseInsensitive) == 0)
        {
            viewportPixmap.save(fileName, "BMP");
            return;
        }

        if (QString::compare(extension, "jpeg", Qt::CaseInsensitive) == 0)
        {
            viewportPixmap.save(fileName, "JPEG");
            return;
        }

        CV_Error(CV_StsNullPtr, "file extension not recognized, please choose between JPG, JPEG, BMP or PNG");
    }
}



void DefaultEinViewPort::resizeEvent(QResizeEvent* evnt)
{
    controlImagePosition();

    //use to compute mouse coordinate, I need to update the ratio here and in resizeEvent
    ratioX = width() / float(image2Draw_mat->cols);
    ratioY = height() / float(image2Draw_mat->rows);

    if (param_keepRatio == EIN_WINDOW_KEEPRATIO)//to keep the same aspect ratio
    {
        QSize newSize = QSize(image2Draw_mat->cols, image2Draw_mat->rows);
        newSize.scale(evnt->size(), Qt::KeepAspectRatio);

        //imageWidth/imageHeight = newWidth/newHeight +/- epsilon
        //ratioX = ratioY +/- epsilon
        //||ratioX - ratioY|| = epsilon
        if (fabs(ratioX - ratioY) * 100 > ratioX) //avoid infinity loop / epsilon = 1% of ratioX
        {
            resize(newSize);

            //move to the middle
            //newSize get the delta offset to place the picture in the middle of its parent
            newSize = (evnt->size() - newSize) / 2;

            move(newSize.width(), newSize.height());
        }
    }

    return QGraphicsView::resizeEvent(evnt);
}


void DefaultEinViewPort::wheelEvent(QWheelEvent* evnt)
{
    scaleView(evnt->delta() / 240.0, evnt->pos());
    viewport()->update();
}


void DefaultEinViewPort::mousePressEvent(QMouseEvent* evnt)
{
    int cv_event = -1, flags = 0;
    QPoint pt = evnt->pos();

    icvmouseHandler(evnt, mouse_down, cv_event, flags);
    icvmouseProcessing(QPointF(pt), cv_event, flags);

    if (param_matrixWorld.m11()>1)
    {
        setCursor(Qt::ClosedHandCursor);
        positionGrabbing = evnt->pos();
    }

    QWidget::mousePressEvent(evnt);
}


void DefaultEinViewPort::mouseReleaseEvent(QMouseEvent* evnt)
{
    int cv_event = -1, flags = 0;
    QPoint pt = evnt->pos();


    if (param_matrixWorld.m11()>1)
        setCursor(Qt::OpenHandCursor);

    QWidget::mouseReleaseEvent(evnt);
}


void DefaultEinViewPort::mouseDoubleClickEvent(QMouseEvent* evnt)
{
    int cv_event = -1, flags = 0;
    QPoint pt = evnt->pos();
    icvmouseHandler(evnt, mouse_dbclick, cv_event, flags);
    icvmouseProcessing(QPointF(pt), cv_event, flags);
    QWidget::mouseDoubleClickEvent(evnt);
}


void DefaultEinViewPort::mouseMoveEvent(QMouseEvent* evnt)
{
    int cv_event = EIN_EVENT_MOUSEMOVE, flags = 0;
    QPoint pt = evnt->pos();


    if (param_matrixWorld.m11() > 1 && evnt->buttons() == Qt::LeftButton)
    {
        QPointF dxy = (pt - positionGrabbing)/param_matrixWorld.m11();
        positionGrabbing = evnt->pos();
        moveView(dxy);
    }


    QWidget::mouseMoveEvent(evnt);
}


void DefaultEinViewPort::paintEvent(QPaintEvent* evnt)
{
    QPainter myPainter(viewport());
    myPainter.setWorldTransform(param_matrixWorld);

    draw2D(&myPainter);

    //Now disable matrixWorld for overlay display
    myPainter.setWorldMatrixEnabled(false);

    //overlay pixel values if zoomed in far enough
    if (param_matrixWorld.m11()*ratioX >= threshold_zoom_img_region &&
        param_matrixWorld.m11()*ratioY >= threshold_zoom_img_region)
    {
        drawImgRegion(&myPainter);
    }

    //in mode zoom/panning
    if (param_matrixWorld.m11() > 1)
    {
        drawViewOverview(&myPainter);
    }

    //for information overlay
    if (drawInfo)
        drawInstructions(&myPainter);


    QGraphicsView::paintEvent(evnt);
}


void DefaultEinViewPort::stopDisplayInfo()
{
    timerDisplay->stop();
    drawInfo = false;
}


inline bool DefaultEinViewPort::isSameSize(IplImage* img1, IplImage* img2)
{
    return img1->width == img2->width && img1->height == img2->height;
}


void DefaultEinViewPort::controlImagePosition()
{
    qreal left, top, right, bottom;

    //after check top-left, bottom right corner to avoid getting "out" during zoom/panning
    param_matrixWorld.map(0,0,&left,&top);

    if (left > 0)
    {
        param_matrixWorld.translate(-left,0);
        left = 0;
    }
    if (top > 0)
    {
        param_matrixWorld.translate(0,-top);
        top = 0;
    }
    //-------

    QSize sizeImage = size();
    param_matrixWorld.map(sizeImage.width(),sizeImage.height(),&right,&bottom);
    if (right < sizeImage.width())
    {
        param_matrixWorld.translate(sizeImage.width()-right,0);
        right = sizeImage.width();
    }
    if (bottom < sizeImage.height())
    {
        param_matrixWorld.translate(0,sizeImage.height()-bottom);
        bottom = sizeImage.height();
    }

    //save corner position
    positionCorners.setTopLeft(QPoint(left,top));
    positionCorners.setBottomRight(QPoint(right,bottom));
    //save also the inv matrix
    matrixWorld_inv = param_matrixWorld.inverted();

    //viewport()->update();
}

void DefaultEinViewPort::moveView(QPointF delta)
{
    param_matrixWorld.translate(delta.x(),delta.y());
    controlImagePosition();
    viewport()->update();
}

//factor is -0.5 (zoom out) or 0.5 (zoom in)
void DefaultEinViewPort::scaleView(qreal factor,QPointF center)
{
    factor/=5;//-0.1 <-> 0.1
    factor+=1;//0.9 <-> 1.1

    //limit zoom out ---
    if (param_matrixWorld.m11()==1 && factor < 1)
        return;

    if (param_matrixWorld.m11()*factor<1)
        factor = 1/param_matrixWorld.m11();


    //limit zoom int ---
    if (param_matrixWorld.m11()>100 && factor > 1)
        return;

    //inverse the transform
    int a, b;
    matrixWorld_inv.map(center.x(),center.y(),&a,&b);

    param_matrixWorld.translate(a-factor*a,b-factor*b);
    param_matrixWorld.scale(factor,factor);

    controlImagePosition();


    if (param_matrixWorld.m11()>1)
        setCursor(Qt::OpenHandCursor);
    else
        unsetCursor();
}


void DefaultEinViewPort::icvmouseHandler(QMouseEvent *evnt, type_mouse_event category, int &cv_event, int &flags)
{
  Qt::KeyboardModifiers modifiers = evnt->modifiers();
  Qt::MouseButtons buttons = evnt->buttons();

  flags = 0;
  if(modifiers & Qt::ShiftModifier)
    flags |= EIN_EVENT_FLAG_SHIFTKEY;
  if(modifiers & Qt::ControlModifier)
    flags |= EIN_EVENT_FLAG_CTRLKEY;
  if(modifiers & Qt::AltModifier)
    flags |= EIN_EVENT_FLAG_ALTKEY;

  if(buttons & Qt::LeftButton)
    flags |= EIN_EVENT_FLAG_LBUTTON;
  if(buttons & Qt::RightButton)
    flags |= EIN_EVENT_FLAG_RBUTTON;
  if(buttons & Qt::MidButton)
    flags |= EIN_EVENT_FLAG_MBUTTON;

  cv_event = EIN_EVENT_MOUSEMOVE;
  switch(evnt->button())
    {
    case Qt::LeftButton:
      cv_event = tableMouseButtons[category][0];
      flags |= EIN_EVENT_FLAG_LBUTTON;
      break;
    case Qt::RightButton:
      cv_event = tableMouseButtons[category][1];
      flags |= EIN_EVENT_FLAG_RBUTTON;
      break;
    case Qt::MidButton:
      cv_event = tableMouseButtons[category][2];
      flags |= EIN_EVENT_FLAG_MBUTTON;
      break;
    default:;
    }
}



void DefaultEinViewPort::icvmouseProcessing(QPointF pt, int cv_event, int flags)
{
  //to convert mouse coordinate
  qreal pfx, pfy;
  matrixWorld_inv.map(pt.x(),pt.y(),&pfx,&pfy);

  mouseCoordinate.rx()=floor(pfx/ratioX);
  mouseCoordinate.ry()=floor(pfy/ratioY);
  if (on_mouse)
    on_mouse( cv_event, mouseCoordinate.x(),
              mouseCoordinate.y(), flags, on_mouse_param );
}



QSize DefaultEinViewPort::sizeHint() const
{
    if(image2Draw_mat)
        return QSize(image2Draw_mat->cols, image2Draw_mat->rows);
    else
        return QGraphicsView::sizeHint();
}


void DefaultEinViewPort::draw2D(QPainter *painter)
{
    image2Draw_qt = QImage(image2Draw_mat->data.ptr, image2Draw_mat->cols, image2Draw_mat->rows,image2Draw_mat->step,QImage::Format_RGB888);
    painter->drawImage(QRect(0,0,viewport()->width(),viewport()->height()), image2Draw_qt, QRect(0,0, image2Draw_qt.width(), image2Draw_qt.height()) );
}

//only if CV_8UC1 or CV_8UC3
void DefaultEinViewPort::drawStatusBar()
{
    if (nbChannelOriginImage!=CV_8UC1 && nbChannelOriginImage!=CV_8UC3)
        return;

    if (mouseCoordinate.x()>=0 &&
        mouseCoordinate.y()>=0 &&
        mouseCoordinate.x()<image2Draw_qt.width() &&
        mouseCoordinate.y()<image2Draw_qt.height())
//  if (mouseCoordinate.x()>=0 && mouseCoordinate.y()>=0)
    {
        QRgb rgbValue = image2Draw_qt.pixel(mouseCoordinate);
    }
}

//accept only CV_8UC1 and CV_8UC8 image for now
void DefaultEinViewPort::drawImgRegion(QPainter *painter)
{
    if (nbChannelOriginImage!=CV_8UC1 && nbChannelOriginImage!=CV_8UC3)
        return;

    double pixel_width = param_matrixWorld.m11()*ratioX;
    double pixel_height = param_matrixWorld.m11()*ratioY;

    qreal offsetX = param_matrixWorld.dx()/pixel_width;
    offsetX = offsetX - floor(offsetX);
    qreal offsetY = param_matrixWorld.dy()/pixel_height;
    offsetY = offsetY - floor(offsetY);

    QSize view = size();
    QVarLengthArray<QLineF, 30> linesX;
    for (qreal _x = offsetX*pixel_width; _x < view.width(); _x += pixel_width )
        linesX.append(QLineF(_x, 0, _x, view.height()));

    QVarLengthArray<QLineF, 30> linesY;
    for (qreal _y = offsetY*pixel_height; _y < view.height(); _y += pixel_height )
        linesY.append(QLineF(0, _y, view.width(), _y));


    QFont f = painter->font();
    int original_font_size = f.pointSize();
    //change font size
    //f.setPointSize(4+(param_matrixWorld.m11()-threshold_zoom_img_region)/5);
    f.setPixelSize(10+(pixel_height-threshold_zoom_img_region)/5);
    painter->setFont(f);


    for (int j=-1;j<height()/pixel_height;j++)//-1 because display the pixels top rows left columns
        for (int i=-1;i<width()/pixel_width;i++)//-1
        {
            // Calculate top left of the pixel's position in the viewport (screen space)
            QPointF pos_in_view((i+offsetX)*pixel_width, (j+offsetY)*pixel_height);

            // Calculate top left of the pixel's position in the image (image space)
            QPointF pos_in_image = matrixWorld_inv.map(pos_in_view);// Top left of pixel in view
            pos_in_image.rx() = pos_in_image.x()/ratioX;
            pos_in_image.ry() = pos_in_image.y()/ratioY;
            QPoint point_in_image(pos_in_image.x() + 0.5f,pos_in_image.y() + 0.5f);// Add 0.5 for rounding

            QRgb rgbValue;
            if (image2Draw_qt.valid(point_in_image))
                rgbValue = image2Draw_qt.pixel(point_in_image);
            else
                rgbValue = qRgb(0,0,0);

            if (nbChannelOriginImage==CV_8UC3)
            {
                //for debug
                /*
                val = tr("%1 %2").arg(point2.x()).arg(point2.y());
                painter->setPen(QPen(Qt::black, 1));
                painter->drawText(QRect(point1.x(),point1.y(),param_matrixWorld.m11(),param_matrixWorld.m11()/2),
                    Qt::AlignCenter, val);
                */
                QString val;

                val = tr("%1").arg(qRed(rgbValue));
                painter->setPen(QPen(Qt::red, 1));
                painter->drawText(QRect(pos_in_view.x(),pos_in_view.y(),pixel_width,pixel_height/3),
                    Qt::AlignCenter, val);

                val = tr("%1").arg(qGreen(rgbValue));
                painter->setPen(QPen(Qt::green, 1));
                painter->drawText(QRect(pos_in_view.x(),pos_in_view.y()+pixel_height/3,pixel_width,pixel_height/3),
                    Qt::AlignCenter, val);

                val = tr("%1").arg(qBlue(rgbValue));
                painter->setPen(QPen(Qt::blue, 1));
                painter->drawText(QRect(pos_in_view.x(),pos_in_view.y()+2*pixel_height/3,pixel_width,pixel_height/3),
                    Qt::AlignCenter, val);

            }

            if (nbChannelOriginImage==CV_8UC1)
            {
                QString val = tr("%1").arg(qRed(rgbValue));
                painter->drawText(QRect(pos_in_view.x(),pos_in_view.y(),pixel_width,pixel_height),
                    Qt::AlignCenter, val);
            }
        }

        painter->setPen(QPen(Qt::black, 1));
        painter->drawLines(linesX.data(), linesX.size());
        painter->drawLines(linesY.data(), linesY.size());

        //restore font size
        f.setPointSize(original_font_size);
        painter->setFont(f);

}

void DefaultEinViewPort::drawViewOverview(QPainter *painter)
{
    QSize viewSize = size();
    viewSize.scale ( 100, 100,Qt::KeepAspectRatio );

    const int margin = 5;

    //draw the image's location
    painter->setBrush(QColor(0, 0, 0, 127));
    painter->setPen(Qt::darkGreen);
    painter->drawRect(QRect(width()-viewSize.width()-margin, 0,viewSize.width(),viewSize.height()));

    //daw the view's location inside the image
    qreal ratioSize = 1/param_matrixWorld.m11();
    qreal ratioWindow = (qreal)(viewSize.height())/(qreal)(size().height());
    painter->setPen(Qt::darkBlue);
    painter->drawRect(QRectF(width()-viewSize.width()-positionCorners.left()*ratioSize*ratioWindow-margin,
        -positionCorners.top()*ratioSize*ratioWindow,
        (viewSize.width()-1)*ratioSize,
        (viewSize.height()-1)*ratioSize)
        );
}

void DefaultEinViewPort::drawInstructions(QPainter *painter)
{
    QFontMetrics metrics = QFontMetrics(font());
    int border = qMax(4, metrics.leading());

    QRect qrect = metrics.boundingRect(0, 0, width() - 2*border, int(height()*0.125),
        Qt::AlignCenter | Qt::TextWordWrap, infoText);
    painter->setRenderHint(QPainter::TextAntialiasing);
    painter->fillRect(QRect(0, 0, width(), qrect.height() + 2*border),
        QColor(0, 0, 0, 127));
    painter->setPen(Qt::white);
    painter->fillRect(QRect(0, 0, width(), qrect.height() + 2*border),
        QColor(0, 0, 0, 127));

    painter->drawText((width() - qrect.width())/2, border,
        qrect.width(), qrect.height(),
        Qt::AlignCenter | Qt::TextWordWrap, infoText);
}


void DefaultEinViewPort::setSize(QSize /*size_*/)
{
}


//////////////////////////////////////////////////////
// OpenGlEinViewPort

#ifdef HAVE_QT_OPENGL

OpenGlEinViewPort::OpenGlEinViewPort(QWidget* _parent, int keep_ratio) : QGLWidget(_parent), size(-1, -1)
{
  mouseCallback = 0;
  mouseData = 0;

  glDrawCallback = 0;
  glDrawData = 0;
}

OpenGlEinViewPort::~OpenGlEinViewPort()
{
}

QWidget* OpenGlEinViewPort::getWidget()
{
    return this;
}

void OpenGlEinViewPort::setMouseCallBack(EinMouseCallback callback, void* param)
{
  mouseCallback = callback;
  mouseData = param;
}


void OpenGlEinViewPort::writeSettings(QSettings& /*settings*/)
{
}

void OpenGlEinViewPort::readSettings(QSettings& /*settings*/)
{
}

double OpenGlEinViewPort::getRatio()
{
  return (double)width() / height();
}

void OpenGlEinViewPort::setRatio(int /*flags*/)
{
}


void OpenGlEinViewPort::updateImage(const Mat /*arr*/)
{
}

void OpenGlEinViewPort::startDisplayInfo(QString /*text*/, int /*delayms*/)
{
}

void OpenGlEinViewPort::setOpenGlDrawCallback(EinOpenGlDrawCallback callback, void* userdata)
{
    glDrawCallback = callback;
    glDrawData = userdata;
}

void OpenGlEinViewPort::makeCurrentOpenGlContext()
{
    makeCurrent();
}

void OpenGlEinViewPort::updateGl()
{
    QGLWidget::updateGL();
}

void OpenGlEinViewPort::initializeGL()
{
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void OpenGlEinViewPort::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void OpenGlEinViewPort::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (glDrawCallback)
        glDrawCallback(glDrawData);
}

void OpenGlEinViewPort::mousePressEvent(QMouseEvent* evnt)
{
    int cv_event = -1, flags = 0;
    QPoint pt = evnt->pos();


    icvmouseHandler(evnt, mouse_down, cv_event, flags);
    icvmouseProcessing(QPointF(pt), cv_event, flags);


    QGLWidget::mousePressEvent(evnt);
}


void OpenGlEinViewPort::mouseReleaseEvent(QMouseEvent* evnt)
{
    int cv_event = -1, flags = 0;
    QPoint pt = evnt->pos();


    icvmouseHandler(evnt, mouse_up, cv_event, flags);
    icvmouseProcessing(QPointF(pt), cv_event, flags);

    icvmouseHandler(evnt, mouse_dbclick, cv_event, flags);
    icvmouseProcessing(QPointF(pt), cv_event, flags);
    QGLWidget::mouseReleaseEvent(evnt);
}


void OpenGlEinViewPort::mouseDoubleClickEvent(QMouseEvent* evnt)
{
    int cv_event = -1, flags = 0;
    QPoint pt = evnt->pos();
    icvmouseHandler(evnt, mouse_dbclick, cv_event, flags);
    icvmouseProcessing(QPointF(pt), cv_event, flags);


    QGLWidget::mouseDoubleClickEvent(evnt);
}


void OpenGlEinViewPort::mouseMoveEvent(QMouseEvent* evnt)
{
    int cv_event = EIN_EVENT_MOUSEMOVE, flags = 0;
    QPoint pt = evnt->pos();

    //icvmouseHandler: pass parameters for cv_event, flags
    icvmouseHandler(evnt, mouse_move, cv_event, flags);
    icvmouseProcessing(QPointF(pt), cv_event, flags);


    QGLWidget::mouseMoveEvent(evnt);
}


void OpenGlEinViewPort::icvmouseHandler(QMouseEvent* evnt, type_mouse_event category, int& cv_event, int& flags)
{
  Qt::KeyboardModifiers modifiers = evnt->modifiers();
  Qt::MouseButtons buttons = evnt->buttons();

  flags = 0;
  if (modifiers & Qt::ShiftModifier)
    flags |= EIN_EVENT_FLAG_SHIFTKEY;
  if (modifiers & Qt::ControlModifier)
    flags |= EIN_EVENT_FLAG_CTRLKEY;
  if (modifiers & Qt::AltModifier)
    flags |= EIN_EVENT_FLAG_ALTKEY;

  if (buttons & Qt::LeftButton)
    flags |= EIN_EVENT_FLAG_LBUTTON;
  if (buttons & Qt::RightButton)
    flags |= EIN_EVENT_FLAG_RBUTTON;
  if (buttons & Qt::MidButton)
    flags |= EIN_EVENT_FLAG_MBUTTON;

  cv_event = EIN_EVENT_MOUSEMOVE;
  switch (evnt->button())
    {
    case Qt::LeftButton:
      cv_event = tableMouseButtons[category][0];
      flags |= EIN_EVENT_FLAG_LBUTTON;
      break;
    case Qt::RightButton:
      cv_event = tableMouseButtons[category][1];
      flags |= EIN_EVENT_FLAG_RBUTTON;
      break;

    case Qt::MidButton:
      cv_event = tableMouseButtons[category][2];
      flags |= EIN_EVENT_FLAG_MBUTTON;
      break;

    default:
      ;
    }
}


void OpenGlEinViewPort::icvmouseProcessing(QPointF pt, int cv_event, int flags)
{
  if (mouseCallback)
    mouseCallback(cv_event, pt.x(), pt.y(), flags, mouseData);
}



QSize OpenGlEinViewPort::sizeHint() const
{
    if (size.width() > 0 && size.height() > 0)
        return size;

    return QGLWidget::sizeHint();
}

void OpenGlEinViewPort::setSize(QSize size_)
{
    size = size_;
    updateGeometry();
}

#endif


