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

#include <algorithm>

#include <Eigen/Geometry>

#include <QClipboard>

#include <ros/package.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TransformStamped.h>

#include "static_tf_plugin.h"

PLUGINLIB_DECLARE_CLASS(rqt_static_tf, StaticTfPlugin, \
  StaticTfPlugin, rqt_gui_cpp::Plugin)

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

StaticTfPlugin::StaticTfPlugin() :
  rqt_gui_cpp::Plugin(),
  widget_(0),
  tfListener(tfBuffer),
  block(false) {
  setObjectName("StaticTfPlugin");
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void StaticTfPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  QStringList argv = context.argv();

  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  ui_.pushButtonRefresh->setIcon(QIcon(QString::fromStdString(
    ros::package::getPath("rqt_static_tf").append(
    "/resource/refresh.png"))));
  
  connect(ui_.comboBoxParentFrame,
    SIGNAL(currentIndexChanged(const QString&)),
    this, SLOT(comboBoxParentFrameIndexChanged(const QString&)));
  connect(ui_.comboBoxChildFrame,
    SIGNAL(currentIndexChanged(const QString&)),
    this, SLOT(comboBoxChildFrameIndexChanged(const QString&)));
  connect(ui_.pushButtonRefresh, SIGNAL(pressed()),
    this, SLOT(buttonRefreshPressed()));
  
  connect(ui_.doubleSpinBoxTranslationX, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxTranslationValueChanged(double)));
  connect(ui_.doubleSpinBoxTranslationY, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxTranslationValueChanged(double)));
  connect(ui_.doubleSpinBoxTranslationZ, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxTranslationValueChanged(double)));
  
  connect(ui_.doubleSpinBoxRotationEulerX, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxRotationEulerValueChanged(double)));
  connect(ui_.doubleSpinBoxRotationEulerY, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxRotationEulerValueChanged(double)));
  connect(ui_.doubleSpinBoxRotationEulerZ, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxRotationEulerValueChanged(double)));
  
  connect(ui_.doubleSpinBoxRotationQuatW, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxRotationQuatValueChanged(double)));
  connect(ui_.doubleSpinBoxRotationQuatX, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxRotationQuatValueChanged(double)));
  connect(ui_.doubleSpinBoxRotationQuatY, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxRotationQuatValueChanged(double)));
  connect(ui_.doubleSpinBoxRotationQuatZ, SIGNAL(valueChanged(double)),
    this, SLOT(doubleSpinBoxRotationQuatValueChanged(double)));
  
  connect(ui_.pushButtonQuatNormalize, SIGNAL(pressed()),
    this, SLOT(pushButtonQuatNormalizePressed()));
  connect(ui_.comboBoxRotationEulerDegRad,
    SIGNAL(currentIndexChanged(const QString&)),
    this, SLOT(comboBoxRotationEulerDegRadIndexChanged(const QString&)));
  connect(ui_.pushButtonCopy, SIGNAL(pressed()),
    this, SLOT(buttonCopyPressed()));
  
  setEnabledForAll(false);
}

void StaticTfPlugin::shutdownPlugin() {
}

void StaticTfPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const {
}

void StaticTfPlugin::restoreSettings(const qt_gui_cpp::Settings&
    plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
}

void StaticTfPlugin::buttonRefreshPressed() {
  std::vector<std::string> frames;
  tfBuffer._getFrameStrings(frames);
  
  frameGraph.clear();
  for (size_t i = 0; i < frames.size(); ++i) {
    std::string parentFrame;
    
    if (tfBuffer._getParent(frames[i], ros::Time::now(), parentFrame))
      frameGraph[parentFrame].push_back(frames[i]);
  }
  
  for (FrameGraph::iterator it = frameGraph.begin();
       it != frameGraph.end(); ++it)
    std::sort(it->second.begin(), it->second.end());
  
  QString oldParentFrame = ui_.comboBoxParentFrame->currentText();
  QString oldChildFrame = ui_.comboBoxChildFrame->currentText();
  ui_.comboBoxParentFrame->clear();
    
  for (FrameGraph::const_iterator it = frameGraph.begin();
      it != frameGraph.end(); ++it)
    ui_.comboBoxParentFrame->addItem(it->first.c_str());

  int parentFrameItem = -1;
  if (!oldParentFrame.isEmpty())
    parentFrameItem = ui_.comboBoxParentFrame->findText(oldParentFrame);
  ui_.comboBoxParentFrame->setCurrentIndex(parentFrameItem >= 0 ?
    parentFrameItem : 0);
  
  int childFrameItem = -1;
  if (!oldChildFrame.isEmpty())
    childFrameItem = ui_.comboBoxChildFrame->findText(oldChildFrame);
  ui_.comboBoxChildFrame->setCurrentIndex(childFrameItem >= 0 ?
    childFrameItem : 0);
}

void StaticTfPlugin::comboBoxParentFrameIndexChanged(const
    QString& text) {
  std::string parentFrame = text.toStdString();
  QString oldChildFrame = ui_.comboBoxChildFrame->currentText();
  ui_.comboBoxChildFrame->clear();
  
  if (!parentFrame.empty()) {
    FrameGraph::const_iterator it = frameGraph.find(parentFrame);
    
    for (size_t i = 0; i < it->second.size(); ++i)
      ui_.comboBoxChildFrame->addItem(it->second[i].c_str());
  }
  
  int childFrameItem = -1;
  if (!oldChildFrame.isEmpty())
    childFrameItem = ui_.comboBoxChildFrame->findText(oldChildFrame);
  ui_.comboBoxChildFrame->setCurrentIndex(childFrameItem >= 0 ?
    childFrameItem : 0);
}

void StaticTfPlugin::comboBoxChildFrameIndexChanged(const
    QString& text) {
  lookupTransform();
}

void StaticTfPlugin::buttonCopyPressed() {
  std::string parentFrame =
    ui_.comboBoxParentFrame->currentText().toStdString();
  std::string childFrame =
    ui_.comboBoxChildFrame->currentText().toStdString();
    
  if (!parentFrame.empty() && !childFrame.empty()) {
    QClipboard* clipboard = QApplication::clipboard();
    
    QString arguments;
    arguments.sprintf("%f %f %f %f %f %f %f %s %s",
      ui_.doubleSpinBoxTranslationX->value(),
      ui_.doubleSpinBoxTranslationY->value(),
      ui_.doubleSpinBoxTranslationZ->value(),
      ui_.doubleSpinBoxRotationQuatX->value(),
      ui_.doubleSpinBoxRotationQuatY->value(),
      ui_.doubleSpinBoxRotationQuatZ->value(),
      ui_.doubleSpinBoxRotationQuatW->value(),
      parentFrame.c_str(),
      childFrame.c_str()
    );
    
    clipboard->setText(arguments);
  }
}

void StaticTfPlugin::doubleSpinBoxTranslationValueChanged(
    double value) {
  if (!block)
    publishTransform();
}

void StaticTfPlugin::doubleSpinBoxRotationEulerValueChanged(
    double value) {
  if (!block) {
    double x = ui_.doubleSpinBoxRotationEulerX->value();
    double y = ui_.doubleSpinBoxRotationEulerY->value();
    double z = ui_.doubleSpinBoxRotationEulerZ->value();
    
    if (ui_.comboBoxRotationEulerDegRad->currentText() == "degrees") {
      x *= M_PI/180.0;
      y *= M_PI/180.0;
      z *= M_PI/180.0;
    }
    
    Eigen::Quaternionf quat =
      Eigen::AngleAxisf(x, Eigen::Vector3f::UnitX())*
      Eigen::AngleAxisf(y,  Eigen::Vector3f::UnitY())*
      Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ());
      
    block = true;
    ui_.doubleSpinBoxRotationQuatW->setValue(quat.w());
    ui_.doubleSpinBoxRotationQuatX->setValue(quat.x());
    ui_.doubleSpinBoxRotationQuatY->setValue(quat.y());
    ui_.doubleSpinBoxRotationQuatZ->setValue(quat.z());
    block = false;
  
    publishTransform();
  }
}

void StaticTfPlugin::doubleSpinBoxRotationQuatValueChanged(
    double value) {
  if (!block) {
    Eigen::Quaternionf quat(
      ui_.doubleSpinBoxRotationQuatW->value(),
      ui_.doubleSpinBoxRotationQuatX->value(),
      ui_.doubleSpinBoxRotationQuatY->value(),
      ui_.doubleSpinBoxRotationQuatZ->value()
    );
    
    Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    if (ui_.comboBoxRotationEulerDegRad->currentText() == "degrees")
      euler *= 180.0/M_PI;
   
    block = true;
    ui_.doubleSpinBoxRotationEulerX->setValue(euler(0));
    ui_.doubleSpinBoxRotationEulerY->setValue(euler(1));
    ui_.doubleSpinBoxRotationEulerZ->setValue(euler(2));
    block = false;
  
    publishTransform();
  }
}

void StaticTfPlugin::pushButtonQuatNormalizePressed() {
  Eigen::Quaternionf quat(
    ui_.doubleSpinBoxRotationQuatW->value(),
    ui_.doubleSpinBoxRotationQuatX->value(),
    ui_.doubleSpinBoxRotationQuatY->value(),
    ui_.doubleSpinBoxRotationQuatZ->value()
  );
  
  quat.normalize();
  
  block = true;
  ui_.doubleSpinBoxRotationQuatW->setValue(quat.w());
  ui_.doubleSpinBoxRotationQuatX->setValue(quat.x());
  ui_.doubleSpinBoxRotationQuatY->setValue(quat.y());
  ui_.doubleSpinBoxRotationQuatZ->setValue(quat.z());
  block = false;
  
  publishTransform();
}

void StaticTfPlugin::comboBoxRotationEulerDegRadIndexChanged(
    const QString& text) {
  double minValue = -M_PI;
  double maxValue = M_PI;
  double step = 0.1;
  
  if (ui_.comboBoxRotationEulerDegRad->currentText() == "degrees") {
    minValue = -180.0;
    maxValue = 180.0;
    step = 1.0;
  }
  
  block = true;
  ui_.doubleSpinBoxRotationEulerX->setRange(minValue, maxValue);
  ui_.doubleSpinBoxRotationEulerY->setRange(minValue, maxValue);
  ui_.doubleSpinBoxRotationEulerZ->setRange(minValue, maxValue);
  
  ui_.doubleSpinBoxRotationEulerX->setSingleStep(step);
  ui_.doubleSpinBoxRotationEulerY->setSingleStep(step);
  ui_.doubleSpinBoxRotationEulerZ->setSingleStep(step);
  block = false;
  
  lookupTransform();
}

void StaticTfPlugin::setEnabledForAll(bool enabled) {
  ui_.doubleSpinBoxTranslationX->setEnabled(enabled);
  ui_.doubleSpinBoxTranslationY->setEnabled(enabled);
  ui_.doubleSpinBoxTranslationZ->setEnabled(enabled);
  
  ui_.doubleSpinBoxRotationEulerX->setEnabled(enabled);
  ui_.doubleSpinBoxRotationEulerY->setEnabled(enabled);
  ui_.doubleSpinBoxRotationEulerZ->setEnabled(enabled);
  
  ui_.doubleSpinBoxRotationQuatW->setEnabled(enabled);
  ui_.doubleSpinBoxRotationQuatX->setEnabled(enabled);
  ui_.doubleSpinBoxRotationQuatY->setEnabled(enabled);
  ui_.doubleSpinBoxRotationQuatZ->setEnabled(enabled);
  
  ui_.comboBoxRotationEulerDegRad->setEnabled(enabled);
  ui_.pushButtonQuatNormalize->setEnabled(enabled);
}

void StaticTfPlugin::lookupTransform() {
  std::string parentFrame =
    ui_.comboBoxParentFrame->currentText().toStdString();
  std::string childFrame =
    ui_.comboBoxChildFrame->currentText().toStdString();
  
  if (!parentFrame.empty() && !childFrame.empty()) {
    geometry_msgs::TransformStamped message;
    
    try {
      message = tfBuffer.lookupTransform(parentFrame, childFrame,
        ros::Time(0));
    }
    catch (tf2::TransformException& exception) {
      ROS_WARN("Failed to lookup transform from [%s] to [%s]: %s",
        parentFrame.c_str(), childFrame.c_str(), exception.what());
      setEnabledForAll(false);
      
      return;
    }
    
    Eigen::Quaternionf quat(
      message.transform.rotation.w,
      message.transform.rotation.x,
      message.transform.rotation.y,
      message.transform.rotation.z
    );
    Eigen::Vector3f euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    if (ui_.comboBoxRotationEulerDegRad->currentText() == "degrees")
      euler *= 180.0/M_PI;

    block = true;
    ui_.doubleSpinBoxTranslationX->setValue(message.transform.translation.x);
    ui_.doubleSpinBoxTranslationY->setValue(message.transform.translation.y);
    ui_.doubleSpinBoxTranslationZ->setValue(message.transform.translation.z);
    
    ui_.doubleSpinBoxRotationEulerX->setValue(euler(0));
    ui_.doubleSpinBoxRotationEulerY->setValue(euler(1));
    ui_.doubleSpinBoxRotationEulerZ->setValue(euler(2));
    
    ui_.doubleSpinBoxRotationQuatW->setValue(message.transform.rotation.w);
    ui_.doubleSpinBoxRotationQuatX->setValue(message.transform.rotation.x);
    ui_.doubleSpinBoxRotationQuatY->setValue(message.transform.rotation.y);
    ui_.doubleSpinBoxRotationQuatZ->setValue(message.transform.rotation.z);
    block = false;
    
    setEnabledForAll(true);
  }
}

void StaticTfPlugin::publishTransform() {
  std::string parentFrame =
    ui_.comboBoxParentFrame->currentText().toStdString();
  std::string childFrame =
    ui_.comboBoxChildFrame->currentText().toStdString();
    
  if (!parentFrame.empty() && !childFrame.empty()) {
    geometry_msgs::TransformStamped message;
    
    message.header.stamp = ros::Time::now();
    
    message.header.frame_id = parentFrame;
    message.child_frame_id = childFrame;
    
    message.transform.translation.x = ui_.doubleSpinBoxTranslationX->value();
    message.transform.translation.y = ui_.doubleSpinBoxTranslationY->value();
    message.transform.translation.z = ui_.doubleSpinBoxTranslationZ->value();
    
    message.transform.rotation.w = ui_.doubleSpinBoxRotationQuatW->value();
    message.transform.rotation.x = ui_.doubleSpinBoxRotationQuatX->value();
    message.transform.rotation.y = ui_.doubleSpinBoxRotationQuatY->value();
    message.transform.rotation.z = ui_.doubleSpinBoxRotationQuatZ->value();
    
    broadcaster.sendTransform(message);
  }
}
