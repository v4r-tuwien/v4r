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

#ifndef Q_MOC_RUN
#include "mainwindow.h"
#include <v4r/keypoints/impl/toString.hpp>
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include <v4r/camera_tracking_and_mapping/Surfel.hh>
#include <v4r/common/impl/DataMatrix2D.hpp>
#endif

Q_DECLARE_METATYPE(cv::Mat)

using namespace std;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), m_ui(new Ui::MainWindow), m_sensor(new Sensor()) {
  m_ui->setupUi(this);

  m_glviewer = new GLViewer(this);
  m_glview = new GLGraphicsView(m_ui->centralWidget);
  m_ui->glView = m_glview;
  m_glview->setGeometry(10, 0, 640, 480);
  m_glview->setViewport(m_glviewer);

  // input signals
  connect(m_glview, SIGNAL(mouse_moved(QMouseEvent *)), m_glviewer, SLOT(mouse_moved(QMouseEvent *)));
  connect(m_glview, SIGNAL(mouse_pressed(QMouseEvent *)), m_glviewer, SLOT(mouse_pressed(QMouseEvent *)));
  connect(m_glview, SIGNAL(key_pressed(QKeyEvent *)), m_glviewer, SLOT(key_pressed(QKeyEvent *)));
  connect(m_glview, SIGNAL(wheel_event(QWheelEvent *)), m_glviewer, SLOT(wheel_event(QWheelEvent *)));

  // sensor signals
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>("pcl::PointCloud<pcl::PointXYZRGB>::Ptr");
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr");
  qRegisterMetaType<v4r::DataMatrix2D<v4r::Surfel>::Ptr>("v4r::DataMatrix2D<v4r::Surfel>::Ptr");
  qRegisterMetaType<cv::Mat_<cv::Vec3b>>("cv::Mat_<cv::Vec3b>");
  qRegisterMetaType<std::vector<Eigen::Matrix4f>>("std::vector<Eigen::Matrix4f>");
  qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");
  qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<uint64_t>("uint64_t");

  connect(m_sensor,
          SIGNAL(new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const cv::Mat_<cv::Vec3b>, const double)),
          m_glviewer,
          SLOT(new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const cv::Mat_<cv::Vec3b>, const double)));
  connect(m_sensor, SIGNAL(new_image(const cv::Mat_<cv::Vec3b>, const double)), m_glviewer,
          SLOT(new_image(const cv::Mat_<cv::Vec3b>, const double)));
  connect(m_sensor, SIGNAL(new_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const double)), m_glviewer,
          SLOT(new_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const double)));
  connect(m_sensor, SIGNAL(new_sf_cloud(const v4r::DataMatrix2D<v4r::Surfel>::Ptr, const double)), m_glviewer,
          SLOT(new_sf_cloud(const v4r::DataMatrix2D<v4r::Surfel>::Ptr, const double)));
  connect(m_sensor, SIGNAL(update_visualization()), m_glviewer, SLOT(update_visualization()));
  connect(m_sensor, SIGNAL(printStatus(const std::string)), this, SLOT(printStatus(const std::string)));

  setWindowTitle(tr("Demo"));
}

MainWindow::~MainWindow() {
  delete m_ui;
  delete m_glviewer;
  delete m_glview;
  delete m_sensor;
}

/**
 * @brief MainWindow::setStartVis
 */
void MainWindow::setStartVis() {
  m_ui->ShowImage->setChecked(true);
  m_ui->ShowDepthMask->setChecked(false);
  m_ui->ShowCameras->setChecked(false);
  m_ui->ShowPointCloud->setChecked(false);

  m_glviewer->showImage(m_ui->ShowImage->isChecked());
  m_glviewer->showCameras(m_ui->ShowCameras->isChecked());
  m_glviewer->showCloud(m_ui->ShowPointCloud->isChecked());
  m_glviewer->showNormals(m_ui->show_normals->isChecked());
  m_sensor->showDepthMask(m_ui->ShowDepthMask->isChecked());

  m_glviewer->resetView();
}

void MainWindow::printStatus(const std::string &_txt) {
  m_ui->statusLabel->setText(_txt.c_str());
}

void MainWindow::on_CamStart_clicked() {
  setStartVis();
  m_sensor->start();
  m_ui->statusLabel->setText("Status: Started camera");
}

void MainWindow::on_CamStop_clicked() {
  m_sensor->stop();
  m_ui->statusLabel->setText("Status: Stopped camera");
}

void MainWindow::on_ResetView_clicked() {
  m_glviewer->resetView();
}

void MainWindow::on_ShowImage_clicked() {
  m_glviewer->showImage(m_ui->ShowImage->isChecked());
}

void MainWindow::on_ShowCameras_clicked() {
  m_glviewer->showCameras(m_ui->ShowCameras->isChecked());
  // m_sensor->showCameras(m_ui->ShowCameras->isChecked());
}

void MainWindow::on_ShowPointCloud_clicked() {
  m_glviewer->showCloud(m_ui->ShowPointCloud->isChecked());
}

void MainWindow::on_Reset_clicked() {
  m_sensor->reset();
  m_ui->statusLabel->setText("Status: Reset tracker");
}

void MainWindow::on_ShowDepthMask_clicked() {
  m_sensor->showDepthMask(m_ui->ShowDepthMask->isChecked());
}

void MainWindow::on_show_normals_clicked() {
  m_glviewer->showNormals(m_ui->show_normals->isChecked());
}

void MainWindow::on_actionExit_triggered() {
  QApplication::exit();
}

void MainWindow::on_nb_smooth_frames_textChanged(const QString &arg1) {
  int nb_smoothed_frames = arg1.toInt();
  if (nb_smoothed_frames > 0 && nb_smoothed_frames < 60)  // we do not allow to smooth longer than 2s => 60 frames
  {
    cout << "[on_nb_smooth_frames_textChanged] " << nb_smoothed_frames << endl;
    m_sensor->setNumberOfSmoothedFrames(nb_smoothed_frames);
  }
}
