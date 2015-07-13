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

#ifndef TF_MARKER_DISPLAY_H
#define TF_MARKER_DISPLAY_H

#include <ros/ros.h>
#include <ros/publisher.h>

#include <geometry_msgs/PoseStamped.h>

#include <rviz/display.h>

namespace rviz {
  class BoolProperty;
  class ColorProperty;
  class Property;
  class QuaternionProperty;
  class RosTopicProperty;
  class TfFrameProperty;
  class VectorProperty;
};

namespace rviz_tf_marker {
  class TFMarker;
    
  class TFMarkerDisplay :
    public rviz::Display {
  Q_OBJECT
  public:
    TFMarkerDisplay();
    virtual ~TFMarkerDisplay();
    
    void setName(const QString& name);
    
  protected:
    rviz::BoolProperty* showDescriptionProperty;
    rviz::BoolProperty* showAxesProperty;
    rviz::BoolProperty* showCrosshairProperty;
    rviz::ColorProperty* crosshairColorProperty;
    rviz::BoolProperty* showVisualAidsProperty;
    rviz::Property* poseProperty;
    rviz::RosTopicProperty* topicProperty;
    rviz::TfFrameProperty* frameProperty;
    rviz::VectorProperty* positionProperty;
    rviz::Property* orientationProperty;
    rviz::Property* eulerProperty;
    rviz::VectorProperty* eulerDegProperty;
    rviz::VectorProperty* eulerRadProperty;
    rviz::QuaternionProperty* quaternionProperty;
    
    boost::shared_ptr<TFMarker> marker;
    
    bool block;
    
    void onInitialize();
    void onEnable();
    void onDisable();
    void reset();
    
    void publish(const geometry_msgs::PoseStamped& message);
    
  protected slots:
    void updateShowDescription();
    void updateShowAxes();
    void updateShowCrosshair();
    void updateCrosshairColor();
    void updateShowVisualAids();
    void updatePose();
    void updatePoseTopic();
    void updateFrame();
    void updatePosition();
    void updateEulerDeg();
    void updateEulerRad();
    void updateQuaternion();
  };
};

#endif
