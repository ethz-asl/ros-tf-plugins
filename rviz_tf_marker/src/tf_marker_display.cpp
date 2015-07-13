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

#include <pluginlib/class_list_macros.h>

#include <OgreMatrix3.h>
#include <OgreResourceGroupManager.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

#include <geometry_msgs/PoseStamped.h>

#include "tf_marker.h"
#include "tf_marker_display.h"

PLUGINLIB_EXPORT_CLASS(rviz_tf_marker::TFMarkerDisplay, rviz::Display)

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerDisplay::TFMarkerDisplay() :
  showDescriptionProperty(
    new rviz::BoolProperty(
      "Show Description",
      true,
      "Whether or not to show the description of the TF Marker.",
      this,
      SLOT(updateShowDescription())
    )
  ),
  showAxesProperty(
    new rviz::BoolProperty(
      "Show Axes",
      false,
      "Whether or not to show the axes of the TF Marker.",
      this,
      SLOT(updateShowAxes())
    )
  ),
  showCrosshairProperty(
    new rviz::BoolProperty(
      "Show Crosshair",
      true,
      "Whether or not to show the crosshair of the TF Marker.",
      this,
      SLOT(updateShowCrosshair())
    )
  ),
  crosshairColorProperty(
    new rviz::ColorProperty(
      "Color",
      Qt::white,
      "Crosshair color of the TF Marker.",
      showCrosshairProperty,
      SLOT(updateCrosshairColor()),
      this
    )
  ),
  showVisualAidsProperty(
    new rviz::BoolProperty(
      "Show Visual Aids",
      false,
      "Whether or not to show visual helpers while moving/rotating the "
        "TF Marker.",
      this,
      SLOT(updateShowVisualAids())
    )
  ),
  poseProperty(
    new rviz::Property(
      "Pose",
      "",
      "The pose of the TF Marker.",
      this
    )
  ),
  topicProperty(
    new rviz::RosTopicProperty(
      "Topic",
      "",
      ros::message_traits::datatype<geometry_msgs::PoseStamped>(),
      "geometry_msgs::PoseStamped topic to publish on.",
      poseProperty,
      SLOT(updatePoseTopic()),
      this
    )
  ),
  frameProperty(
    new rviz::TfFrameProperty(
      "Frame",
      rviz::TfFrameProperty::FIXED_FRAME_STRING,
      "TF frame into which the TF Marker's pose is transformed before being "
        "published.",
      poseProperty,
      0,
      true,
      SLOT(updateFrame()),
      this
    )
  ),
  positionProperty(
    new rviz::VectorProperty(
      "Position",
      Ogre::Vector3::ZERO,
      "Position of the TF Marker in the specified TF frame.",
      poseProperty,
      SLOT(updatePosition()),
      this
    )
  ),
  orientationProperty(
    new rviz::Property(
      "Orientation",
      "",
      "Orientation of the TF Marker in the specified TF frame.",
      poseProperty
    )
  ),
  eulerProperty(
    new rviz::Property(
      "Euler Angles",
      "",
      "Euler angle orientation of the TF Marker in the specified TF frame.",
      orientationProperty
    )
  ),
  eulerDegProperty(
    new rviz::VectorProperty(
      "Degrees",
      Ogre::Vector3::ZERO,
      "Euler angle orientation of the TF Marker in the specified TF frame "
        "in [deg].",
      eulerProperty,
      SLOT(updateEulerDeg()),
      this
    )
  ),
  eulerRadProperty(
    new rviz::VectorProperty(
      "Radians",
      Ogre::Vector3::ZERO,
      "Euler angle orientation of the TF Marker in the specified TF frame "
        "in [rad].",
      eulerProperty,
      SLOT(updateEulerRad()),
      this
    )
  ),
  quaternionProperty(
    new rviz::QuaternionProperty(
      "Quaternion",
      Ogre::Quaternion::IDENTITY,
      "Quaternion orientation of the TF Marker in the specified TF frame.",
      orientationProperty,
      SLOT(updateQuaternion()),
      this
    )
  ),
  block(false) {
}

TFMarkerDisplay::~TFMarkerDisplay() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void TFMarkerDisplay::setName(const QString& name) {
  Display::setName(name);
  marker->setDescription(name);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void TFMarkerDisplay::onInitialize() {
  Ogre::ResourceGroupManager::getSingleton().createResourceGroup(
    "rviz_tf_marker");
  
  marker.reset(new TFMarker(context_, getSceneNode()));
  
  marker->setDescription(getName());

  updateShowDescription();
  updateShowAxes();
  updateShowCrosshair();
  updateCrosshairColor();
  updateShowVisualAids();
  updatePose();
}

void TFMarkerDisplay::onEnable() {
  updateShowAxes();
}

void TFMarkerDisplay::onDisable() {
}

void TFMarkerDisplay::reset() {
  Display::reset();
}

void TFMarkerDisplay::publish(const geometry_msgs::PoseStamped& message) {
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void TFMarkerDisplay::updateShowDescription() {
  marker->setShowDescription(showDescriptionProperty->getBool());
}

void TFMarkerDisplay::updateShowAxes() {
  marker->setShowAxes(showAxesProperty->getBool());
}

void TFMarkerDisplay::updateShowCrosshair() {
  marker->setShowCrosshair(showCrosshairProperty->getBool());
}

void TFMarkerDisplay::updateCrosshairColor() {
  marker->setCrosshairColor(crosshairColorProperty->getColor());
}

void TFMarkerDisplay::updateShowVisualAids() {
  marker->setShowVisualAids(showVisualAidsProperty->getBool());
}

void TFMarkerDisplay::updatePose() {
  if (!block) {
    geometry_msgs::PoseStamped message;
    
    message.header.stamp = ros::Time::now();
    message.header.frame_id = frameProperty->getFrame().toStdString();
    
    Ogre::Vector3 position = positionProperty->getVector();
    message.pose.position.x = position[0];
    message.pose.position.y = position[1];
    message.pose.position.z = position[2];
    
    Ogre::Quaternion orientation = quaternionProperty->getQuaternion();
    message.pose.orientation.w = orientation[0];
    message.pose.orientation.x = orientation[1];
    message.pose.orientation.y = orientation[2];
    message.pose.orientation.z = orientation[3];
    
    marker->setPose(message);
    publish(message);
  }
}

void TFMarkerDisplay::updatePoseTopic() {
}

void TFMarkerDisplay::updateFrame() {
  updatePose();
}

void TFMarkerDisplay::updatePosition() {
  updatePose();
}

void TFMarkerDisplay::updateEulerDeg() {
  if (!block) {  
    Ogre::Vector3 eulerDeg = eulerDegProperty->getVector();
    Ogre::Vector3 eulerRad = eulerDeg*Ogre::Math::HALF_PI/180.0;

    Ogre::Matrix3 matrix = Ogre::Matrix3::IDENTITY;
    matrix.FromEulerAnglesZYX(
      Ogre::Radian(eulerRad[2]), 
      Ogre::Radian(eulerRad[1]),
      Ogre::Radian(eulerRad[0])
    );
    Ogre::Quaternion quaternion(matrix);
    
    block = true;
    eulerRadProperty->setVector(eulerRad);
    quaternionProperty->setQuaternion(quaternion);
    block = false;
  }

  updatePose();
}

void TFMarkerDisplay::updateEulerRad() {
  if (!block) {  
    Ogre::Vector3 eulerRad = eulerRadProperty->getVector();
    Ogre::Vector3 eulerDeg = eulerRad*180.0/Ogre::Math::HALF_PI;
    
    Ogre::Matrix3 matrix = Ogre::Matrix3::IDENTITY;
    matrix.FromEulerAnglesXYZ(
      Ogre::Radian(eulerRad[0]), 
      Ogre::Radian(eulerRad[1]),
      Ogre::Radian(eulerRad[2])
    );
    Ogre::Quaternion quaternion(matrix);
    
    block = true;
    eulerDegProperty->setVector(eulerDeg);
    quaternionProperty->setQuaternion(quaternion);
    block = false;
  }
  
  updatePose();
}

void TFMarkerDisplay::updateQuaternion() {
  if (!block) {
    Ogre::Quaternion quaternion = quaternionProperty->getQuaternion();
    Ogre::Matrix3 matrix = Ogre::Matrix3::IDENTITY;
    quaternion.ToRotationMatrix(matrix);
    
    Ogre::Radian yaw, pitch, roll;
    matrix.ToEulerAnglesZYX(yaw, pitch, roll);
    
    Ogre::Vector3 eulerRad(
      roll.valueRadians(),
      pitch.valueRadians(),
      yaw.valueRadians()
    );
    Ogre::Vector3 eulerDeg = eulerRad*180.0/Ogre::Math::HALF_PI;
    
    block = true;
    eulerDegProperty->setVector(eulerDeg);
    eulerRadProperty->setVector(eulerRad);
    block = false;
  }
    
  updatePose();
}

}
