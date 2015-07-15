/******************************************************************************
 * Copyright (C) 2015 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#ifndef STATIC_TF_PLUGIN_H
#define STATIC_TF_PLUGIN_H

#include <map>
#include <vector>

#include <QWidget>
#include <QInputDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QTimer>

#include <ros/ros.h>
#include <ros/service_client.h>

#include <rqt_gui_cpp/plugin.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ui_static_tf_plugin.h>

namespace rqt_static_tf {

class StaticTFPlugin :
  public rqt_gui_cpp::Plugin {
Q_OBJECT
public:
  StaticTFPlugin();
  
  void initPlugin(qt_gui_cpp::PluginContext& context);
  void shutdownPlugin();
  void saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const;
  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings);

protected slots:
  void buttonRefreshPressed();
  void comboBoxParentFrameIndexChanged(const QString& text);
  void comboBoxChildFrameIndexChanged(const QString& text);
  void doubleSpinBoxTranslationValueChanged(double value);
  void doubleSpinBoxRotationEulerValueChanged(double value);
  void doubleSpinBoxRotationQuatValueChanged(double value);
  void pushButtonQuatNormalizePressed();
  void comboBoxRotationEulerDegRadIndexChanged(const QString& text);
  void buttonCopyPressed();

private:
  typedef std::map<std::string, std::vector<std::string> > FrameGraph;
  
  Ui::static_tf_plugin ui_;
  QWidget* widget_;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  tf2_ros::StaticTransformBroadcaster broadcaster;
  
  FrameGraph frameGraph;
  
  bool block;
  
  void setEnabledForAll(bool enabled);
  void lookupTransform();
  void publishTransform();
};

};

#endif
