/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#include "glviewer.h"

using namespace std;

GLViewer::GLViewer(QWidget *_parent)
: QGLWidget(_parent), m_width(640), m_height(480), m_point_size(1.), m_timer(this), ts_cloud(0), ts_image(0),
  ts_sf_cloud(0), m_show_image(true), m_show_cloud(false), m_show_cameras(false), m_show_normals(false),
  cor(glm::vec3(0., 0., 0.)) {
  setAttribute(Qt::WA_NoSystemBackground, true);
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setCursor(Qt::PointingHandCursor);

  connect(&m_timer, SIGNAL(timeout()), this, SLOT(draw()));
  m_timer.start(200);
  //  m_elapsed.start();

  pt00 = Eigen::Vector4f(0., 0., 0., 1.);
  pt0x = Eigen::Vector4f(1., 0., 0., 1.);
  pt0y = Eigen::Vector4f(0., 1., 0., 1.);
  pt0z = Eigen::Vector4f(0., 0., 1., 1.);

  m_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  m_sf_cloud.reset(new v4r::DataMatrix2D<v4r::Surfel>());

  initCamera();
}

GLViewer::~GLViewer() {}

void GLViewer::resetView(float fw) {
  m_cam_perspective = m_cam_origin;
  if (fabs(fw) > 0.001) {
    m_cam_perspective.TranslateForward(fw);
  }
  updateGL();
}

void GLViewer::showImage(bool enable) {
  m_show_image = enable;
}

void GLViewer::showCloud(bool enable) {
  m_show_cloud = enable;
}

void GLViewer::showCameras(bool enable) {
  m_show_cameras = enable;
  // cout<<"[GLViewer::showCameras] "<<m_show_cameras<<endl;
}

void GLViewer::showNormals(bool enable) {
  m_show_normals = enable;
}

void GLViewer::new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const cv::Mat_<cv::Vec3b> &_image,
                         const double &_ts) {
  *m_cloud = *_cloud;
  _image.copyTo(m_image);
  ts_cloud = _ts;
  ts_image = _ts;
}

void GLViewer::new_image(const cv::Mat_<cv::Vec3b> &_image, const double &_ts) {
  _image.copyTo(m_image);
  ts_image = _ts;
}

void GLViewer::new_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const double &_ts) {
  *m_cloud = *_cloud;
  ts_cloud = _ts;
}

void GLViewer::new_sf_cloud(const v4r::DataMatrix2D<v4r::Surfel>::Ptr &_sf_cloud, const double &_ts) {
  *m_sf_cloud = *_sf_cloud;
  ts_sf_cloud = _ts;
}

void GLViewer::update_visualization() {
  updateGL();
}

void GLViewer::draw() {
  updateGL();
}

void GLViewer::mouse_moved(QMouseEvent *event) {
  this->mouseMoveEvent(event);
}

void GLViewer::mouse_pressed(QMouseEvent *event) {
  this->mousePressEvent(event);
}

void GLViewer::key_pressed(QKeyEvent *event) {
  this->keyPressEvent(event);
}

void GLViewer::wheel_event(QWheelEvent *_event) {
  this->wheelEvent(_event);
}

void GLViewer::initializeGL() {
  glGenTextures(1, &m_texture_id);
  glBindTexture(GL_TEXTURE_2D, m_texture_id);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  glClearColor(0.5, 0.5, 0.5, 0);
}

void GLViewer::resizeGL(int w, int h) {
  (void)w;
  (void)h;
  // cout << "[GLViewer::resizeGL] w,h: " << width() << ", " << height() << endl;
  glViewport(0, 0, width(), height());
  updateGL();
}

void GLViewer::initCamera() {
  size_t size[2];
  float f[2];
  float c[2];
  float range[2];

  cv::Mat_<float> ext = cv::Mat_<float>::eye(4, 4);
  m_width = size[0] = 640;
  m_height = size[1] = 480;
  f[0] = 525;
  f[1] = 525;
  c[0] = 320;
  c[1] = 240;
  range[0] = 0.01;
  range[1] = 5;

  this->setGeometry(0, 0, m_width, m_height);

  glm::mat4 E;
  E[0][0] = ext.at<float>(0, 0);
  E[0][1] = ext.at<float>(0, 1);
  E[0][2] = ext.at<float>(0, 2);
  E[0][3] = ext.at<float>(0, 3);
  E[1][0] = ext.at<float>(1, 0);
  E[1][1] = ext.at<float>(1, 1);
  E[1][2] = ext.at<float>(1, 2);
  E[1][3] = ext.at<float>(1, 3);
  E[2][0] = ext.at<float>(2, 0);
  E[2][1] = ext.at<float>(2, 1);
  E[2][2] = ext.at<float>(2, 2);
  E[2][3] = ext.at<float>(2, 3);
  E[3][0] = ext.at<float>(3, 0);
  E[3][1] = ext.at<float>(3, 1);
  E[3][2] = ext.at<float>(3, 2);
  E[3][3] = ext.at<float>(3, 3);

  m_cam_perspective.SetPerspective(f[0], f[1], c[0], c[1], size[0], size[1], 0.5f * range[0], 2.0f * range[1]);
  m_cam_perspective.SetExtrinsic(tg::Camera::cv2gl(E));
  m_cam_origin = m_cam_perspective;
  m_cam_ortho.SetOrtho(size[0], size[1], 0.1f, 2.0f);

  updateGL();
}

void GLViewer::drawCoordinates(float length, const Eigen::Matrix4f &pose) {
  (void)length;
  pt10 = pose * pt00;
  pt1x = pose * pt0x;
  pt1y = pose * pt0y;
  pt1z = pose * pt0z;

  m_cam_perspective.Activate();

  glDisable(GL_LIGHTING);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glColor3ub(255, 0, 0);
  glVertex3f(pt10[0], pt10[1], pt10[2]);
  glVertex3f(pt1x[0], pt1x[1], pt1x[2]);
  glColor3ub(0, 255, 0);
  glVertex3f(pt10[0], pt10[1], pt10[2]);
  glVertex3f(pt1y[0], pt1y[1], pt1y[2]);
  glColor3ub(0, 0, 255);
  glVertex3f(pt10[0], pt10[1], pt10[2]);
  glVertex3f(pt1z[0], pt1z[1], pt1z[2]);
  glEnd();
}

void GLViewer::drawImage() {
  // draw image
  if (m_show_image && !m_image.empty()) {
    m_cam_ortho.Activate();

    glBindTexture(GL_TEXTURE_2D, m_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_image.cols, m_image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, m_image.data);

    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    float w = float(m_width);
    float h = float(m_height);
    glColor3f(1, 1, 1);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(0, 0, 0.0f);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(w, 0, 0.0f);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(w, h, 0.0f);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(0, h, 0.0f);
    glEnd();

    glEnable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    glBindTexture(GL_TEXTURE_2D, 0);
  }

  // draw point cloud
  if (m_show_cloud) {
    if (ts_cloud > ts_sf_cloud) {
      if (m_cloud.get() != 0) {
        m_cam_perspective.Activate();
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        glPointSize(m_point_size);
        const pcl::PointCloud<pcl::PointXYZRGB> &_cloud = *m_cloud;
        glBegin(GL_POINTS);
        for (unsigned i = 0; i < _cloud.points.size(); i++) {
          const pcl::PointXYZRGB &pt = _cloud.points[i];
          if (!std::isnan(pt.x)) {
            glColor3ub(pt.r, pt.g, pt.b);
            glVertex3f(pt.x, pt.y, pt.z);
          }
        }
        glEnd();
        glEnable(GL_LIGHTING);
      }
    } else {
      if (m_sf_cloud.get() != 0) {
        m_cam_perspective.Activate();
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        glPointSize(m_point_size);
        const v4r::DataMatrix2D<v4r::Surfel> &_sf_cloud = *m_sf_cloud;
        glBegin(GL_POINTS);
        for (unsigned i = 0; i < _sf_cloud.data.size(); i++) {
          const v4r::Surfel &sf = _sf_cloud.data[i];
          if (!std::isnan(sf.pt[0])) {
            glColor3ub(sf.r, sf.g, sf.b);
            glVertex3f(sf.pt[0], sf.pt[1], sf.pt[2]);
          }
        }
        glEnd();
        if (m_show_normals) {
          Eigen::Vector3f pt2;
          glBegin(GL_LINES);
          for (unsigned i = 0; i < _sf_cloud.data.size(); i++) {
            const v4r::Surfel &sf = _sf_cloud.data[i];
            pt2 = sf.pt + 0.003 * sf.n;
            glColor3f(1., 1., 1.);
            glVertex3f(sf.pt[0], sf.pt[1], sf.pt[2]);
            glVertex3f(pt2[0], pt2[1], pt2[2]);
          }
          glEnd();
        }
        glEnable(GL_LIGHTING);
      }
    }
  }

  // draw camera trajectory
  if (m_show_cameras) {
    //    cam_mutex.lock();
    //    Eigen::Vector3f pt;
    //    m_cam_perspective.Activate();

    //    glDisable(GL_TEXTURE_2D);
    //    glDisable(GL_LIGHTING);
    //    glEnable(GL_DEPTH_TEST);

    //    const std::vector<Sensor::CameraLocation> &traj = cam_trajectory;

    //    glLineWidth(1);

    //    glBegin(GL_LINE_STRIP);
    //    glColor3f(.8, .8, .8);
    //    for (unsigned i = 0; i < traj.size(); i++) {
    //      const Sensor::CameraLocation &t0 = traj[i];
    //      glVertex3f(t0.pt[0], t0.pt[1], t0.pt[2]);
    //    }
    //    glEnd();

    //    for (unsigned i = 0; i < traj.size(); i++) {
    //      const Sensor::CameraLocation &t = traj[i];

    //      if (t.type == 0) {
    //        glPointSize(2.0f);
    //        glBegin(GL_POINTS);
    //        glColor3ub(255, 255, 0);
    //        glVertex3f(t.pt[0], t.pt[1], t.pt[2]);
    //        glEnd();
    //      } else if (t.type == 1) {
    //        glPointSize(2.0f);
    //        glBegin(GL_POINTS);
    //        glColor3ub(0, 0, 255);
    //        glVertex3f(t.pt[0], t.pt[1], t.pt[2]);
    //        glEnd();
    //      } else if (t.type == 2) {
    //        pt = t.pt + 0.05 * t.vr;

    //        glPointSize(6.0f);
    //        glBegin(GL_POINTS);
    //        glColor3ub(0, 255, 0);
    //        glVertex3f(t.pt[0], t.pt[1], t.pt[2]);
    //        glEnd();

    //        glBegin(GL_LINES);
    //        glColor3f(1., 1., 1.);
    //        glVertex3f(t.pt[0], t.pt[1], t.pt[2]);
    //        glVertex3f(pt[0], pt[1], pt[2]);
    //        glEnd();
    //      }
    //    }

    //    glEnable(GL_LIGHTING);
    //    cam_mutex.unlock();
  }
}

void GLViewer::paintGL() {
  // enable GL context
  makeCurrent();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawImage();

  // drawCoordinates(0.25f);
}

void GLViewer::mousePressEvent(QMouseEvent *event) {
  //  cout << "GLWidget::mousePressEvent" << endl;
  m_last_point_2d = event->pos();

  //  if (!m_show_cloud && m_segment_object) {
  //    emit segment_image(m_last_point_2d.x(), m_last_point_2d.y());
  //  }

  updateGL();
}

void GLViewer::mouseMoveEvent(QMouseEvent *event) {
  // enable GL context
  makeCurrent();

  QPoint newPoint2D = event->pos();

  float dx = newPoint2D.x() - m_last_point_2d.x();
  float dy = newPoint2D.y() - m_last_point_2d.y();

  float far = m_cam_perspective.GetFar();
  float near = m_cam_perspective.GetNear();

  // move in z direction
  if ((event->buttons() == Qt::MidButton)) {
    m_cam_perspective.TranslateForward(0.0001f * (far - near) * dx);
    m_cam_perspective.TranslateForward(0.0001f * (far - near) * dy);
  }  // move in x,y direction
  else if ((event->buttons() == Qt::RightButton)) {
    m_cam_perspective.TranslateSideward(0.0001f * (far - near) * dx);
    m_cam_perspective.TranslateUpward(-0.0001f * (far - near) * dy);
  }  // rotate
  else if (event->buttons() == Qt::LeftButton) {
    m_cam_perspective.Orbit(cor, m_cam_perspective.GetUpward(), -0.05f * dx);
    m_cam_perspective.Orbit(cor, m_cam_perspective.GetSideward(), -0.05f * dy);
  }

  // remember this point
  m_last_point_2d = newPoint2D;

  // trigger redraw
  updateGL();
}

void GLViewer::wheelEvent(QWheelEvent *event) {
  float d = -(float)event->delta() / 120.0;

  float far = m_cam_perspective.GetFar();
  float near = m_cam_perspective.GetNear();
  m_cam_perspective.TranslateForward(0.0001f * (far - near) * d);
  m_cam_perspective.TranslateForward(0.0001f * (far - near) * d);

  updateGL();
  event->accept();
}

void GLViewer::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Z) {
    m_cam_perspective = m_cam_origin;
    m_cam_perspective.Activate();
  }
  if (event->key() == Qt::Key_O) {
    m_cam_perspective.Print();
    std::cout << std::endl;
  }
  if (event->key() == Qt::Key_Plus) {
    if (m_point_size < 50)
      m_point_size += 1.;
  }
  if (event->key() == Qt::Key_Minus) {
    if (m_point_size > 0)
      m_point_size -= 1.;
  }

  updateGL();
}
