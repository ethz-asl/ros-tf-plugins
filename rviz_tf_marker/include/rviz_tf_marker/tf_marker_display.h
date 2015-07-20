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

#ifndef Q_MOC_RUN
#include <OgreQuaternion.h>
#include <OgreMatrix3.h>
#include <OgreVector3.h>

#include <ros/ros.h>
#include <ros/publisher.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rviz/display.h>
#endif

namespace rviz {
  class BoolProperty;
  class ColorProperty;
  class EnumProperty;
  class FloatProperty;
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
    QString getFixedFrame() const;
    
  signals:
    void initialized();
    void nameChanged(const QString& name);
  
  protected:
    rviz::BoolProperty* showDescriptionProperty;
    rviz::ColorProperty* descriptionColorProperty;
    rviz::BoolProperty* showAxesProperty;
    rviz::BoolProperty* showCrosshairProperty;
    rviz::ColorProperty* crosshairColorProperty;
    rviz::FloatProperty* crosshairAlphaProperty;
    rviz::BoolProperty* showControlsProperty;
    rviz::BoolProperty* showPositionControlsProperty;
    rviz::BoolProperty* showXControlsProperty;
    rviz::BoolProperty* showYControlsProperty;
    rviz::BoolProperty* showZControlsProperty;
    rviz::BoolProperty* showOrientationControlsProperty;
    rviz::BoolProperty* showYawControlsProperty;
    rviz::EnumProperty* yawControlModeProperty;
    rviz::BoolProperty* showPitchControlsProperty;
    rviz::EnumProperty* pitchControlModeProperty;
    rviz::BoolProperty* showRollControlsProperty;
    rviz::EnumProperty* rollControlModeProperty;
    rviz::BoolProperty* showVisualAidsProperty;
    rviz::BoolProperty* enableTransparencyProperty;
    rviz::Property* poseProperty;
    rviz::RosTopicProperty* topicProperty;
    rviz::TfFrameProperty* frameProperty;
    rviz::VectorProperty* positionProperty;
    rviz::Property* orientationProperty;
    rviz::Property* eulerProperty;
    rviz::VectorProperty* eulerDegProperty;
    rviz::VectorProperty* eulerRadProperty;
    rviz::QuaternionProperty* quaternionProperty;
    rviz::BoolProperty* trackUpdatesProperty;
    
    boost::shared_ptr<TFMarker> marker;
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher publisher;
    
    QString frame;
    
    bool block;
    
    void onInitialize();
    void onEnable();
    void onDisable();
    void reset();
    
    bool transform(const geometry_msgs::PoseStamped& message, 
      geometry_msgs::PoseStamped& transformedMessage, const
      QString& targetFrame);
    void publish(const geometry_msgs::PoseStamped& message);
    
    void fromMessage(const geometry_msgs::PoseStamped& message);
    void toMessage(geometry_msgs::PoseStamped& message,
      const Ogre::Vector3& position, const Ogre::Quaternion&
      orientation, const QString& frame, const ros::Time&
      stamp = ros::Time::now());
    void toMessage(geometry_msgs::PoseStamped& message,
      const ros::Time& stamp = ros::Time::now());
    
    void quaternionToEulerRad(const Ogre::Quaternion& quaternion,
      Ogre::Vector3& eulerRad);
    void quaternionToEulerDeg(const Ogre::Quaternion& quaternion,
      Ogre::Vector3& eulerDeg);
    void eulerRadToQuaternion(const Ogre::Vector3& eulerRad,
      Ogre::Quaternion& quaternion);
    void eulerDegToQuaternion(const Ogre::Vector3& eulerDeg,
      Ogre::Quaternion& quaternion);
    void eulerRadToEulerDeg(const Ogre::Vector3& eulerRad,
      Ogre::Vector3& eulerDeg);
    void eulerDegToEulerRad(const Ogre::Vector3& eulerDeg,
      Ogre::Vector3& eulerRad);
    
  protected slots:
    void updateShowDescription();
    void updateDescriptionColor();
    void updateShowAxes();
    void updateShowCrosshair();
    void updateCrosshairColor();
    void updateCrosshairAlpha();
    void updateShowControls();
    void updateShowPositionControls();
    void updateShowXControls();
    void updateShowYControls();
    void updateShowZControls();
    void updateShowOrientationControls();
    void updateShowYawControls();
    void updateYawControlMode();
    void updateShowPitchControls();
    void updatePitchControlMode();
    void updateShowRollControls();
    void updateRollControlMode();
    void updateShowVisualAids();
    void updateEnableTransparency();
    void updatePose();
    void updateTopic();
    void updateFrame();
    void updatePosition();
    void updateEulerDeg();
    void updateEulerRad();
    void updateQuaternion();
    void updateTrackUpdates();
    
    void markerPoseChanged(const Ogre::Vector3& position, const
      Ogre::Quaternion& orientation);
    void markerDraggingStopped();
  };
};

#endif
