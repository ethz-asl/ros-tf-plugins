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

#include <OgreResourceGroupManager.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_manager.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rviz_tf_marker/tf_marker.h"
#include "rviz_tf_marker/tf_marker_display.h"

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
      "Whether or not to show the description of the TF marker.",
      this,
      SLOT(updateShowDescription())
    )
  ),
  descriptionColorProperty(
    new rviz::ColorProperty(
      "Color",
      Qt::white,
      "Description color of the TF marker.",
      showDescriptionProperty,
      SLOT(updateDescriptionColor()),
      this
    )
  ),
  showAxesProperty(
    new rviz::BoolProperty(
      "Show Axes",
      false,
      "Whether or not to show the axes of the TF marker.",
      this,
      SLOT(updateShowAxes())
    )
  ),
  showCrosshairProperty(
    new rviz::BoolProperty(
      "Show Crosshair",
      true,
      "Whether or not to show the crosshair of the TF marker.",
      this,
      SLOT(updateShowCrosshair())
    )
  ),
  crosshairColorProperty(
    new rviz::ColorProperty(
      "Color",
      Qt::white,
      "Crosshair color of the TF marker.",
      showCrosshairProperty,
      SLOT(updateCrosshairColor()),
      this
    )
  ),
  showControlsProperty(
    new rviz::BoolProperty(
      "Show Controls",
      true,
      "Whether or not to show controls for moving/rotating the TF marker.",
      this,
      SLOT(updateShowControls())
    )
  ),
  showPositionControlsProperty(
    new rviz::BoolProperty(
      "Position",
      true,
      "Whether or not to show controls for moving the TF marker.",
      showControlsProperty,
      SLOT(updateShowPositionControls()),
      this
    )
  ),
  showXControlsProperty(
    new rviz::BoolProperty(
      "X",
      true,
      "Whether or not to show controls for moving the TF marker "
        "along the X-axis.",
      showPositionControlsProperty,
      SLOT(updateShowXControls()),
      this
    )
  ),
  showYControlsProperty(
    new rviz::BoolProperty(
      "Y",
      true,
      "Whether or not to show controls for moving the TF marker "
        "along the Y-axis.",
      showPositionControlsProperty,
      SLOT(updateShowYControls()),
      this
    )
  ),
  showZControlsProperty(
    new rviz::BoolProperty(
      "Z",
      true,
      "Whether or not to show controls for moving the TF marker "
        "along the Z-axis.",
      showPositionControlsProperty,
      SLOT(updateShowZControls()),
      this
    )
  ),
  showOrientationControlsProperty(
    new rviz::BoolProperty(
      "Orientation",
      true,
      "Whether or not to show controls for rotating the TF marker.",
      showControlsProperty,
      SLOT(updateShowOrientationControls()),
      this
    )
  ),
  showYawControlsProperty(
    new rviz::BoolProperty(
      "Yaw",
      true,
      "Whether or not to show the control for rotating the TF marker "
        "about the Z-axis.",
      showOrientationControlsProperty,
      SLOT(updateShowYawControls()),
      this
    )
  ),
  showPitchControlsProperty(
    new rviz::BoolProperty(
      "Pitch",
      true,
      "Whether or not to show the control for rotating the TF marker "
        "about the Y-axis.",
      showOrientationControlsProperty,
      SLOT(updateShowPitchControls()),
      this
    )
  ),
  showRollControlsProperty(
    new rviz::BoolProperty(
      "Roll",
      true,
      "Whether or not to show the control for rotating the TF marker "
        "about the X-axis.",
      showOrientationControlsProperty,
      SLOT(updateShowRollControls()),
      this
    )
  ),
  poseProperty(
    new rviz::Property(
      "Pose",
      "",
      "The pose of the TF marker.",
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
      SLOT(updateTopic()),
      this
    )
  ),
  frameProperty(
    new rviz::TfFrameProperty(
      "Frame",
      rviz::TfFrameProperty::FIXED_FRAME_STRING,
      "TF frame into which the TF marker's pose is transformed before being "
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
      "Position of the TF marker in the specified TF frame.",
      poseProperty,
      SLOT(updatePosition()),
      this
    )
  ),
  orientationProperty(
    new rviz::Property(
      "Orientation",
      "",
      "Orientation of the TF marker in the specified TF frame.",
      poseProperty
    )
  ),
  eulerProperty(
    new rviz::Property(
      "Euler Angles",
      "",
      "Euler angle orientation of the TF marker in the specified TF frame.",
      orientationProperty
    )
  ),
  eulerDegProperty(
    new rviz::VectorProperty(
      "Degrees",
      Ogre::Vector3::ZERO,
      "Euler angle orientation of the TF marker in the specified TF frame "
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
      "Euler angle orientation of the TF marker in the specified TF frame "
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
      "Quaternion orientation of the TF marker in the specified TF frame.",
      orientationProperty,
      SLOT(updateQuaternion()),
      this
    )
  ),
  tfListener(tfBuffer),
  frame(rviz::TfFrameProperty::FIXED_FRAME_STRING),
  block(false) {
  showDescriptionProperty->setDisableChildrenIfFalse(true);
  showCrosshairProperty->setDisableChildrenIfFalse(true);
  showControlsProperty->setDisableChildrenIfFalse(true);
  showPositionControlsProperty->setDisableChildrenIfFalse(true);
  showOrientationControlsProperty->setDisableChildrenIfFalse(true);
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
  frameProperty->setFrameManager(context_->getFrameManager());
  Ogre::ResourceGroupManager::getSingleton().createResourceGroup(
    "rviz_tf_marker");
  
  marker.reset(new TFMarker(context_, getSceneNode(), this));
  
  marker->setDescription(getName());

  updateShowDescription();
  updateShowAxes();
  updateShowCrosshair();
  updateCrosshairColor();
  updateShowControls();
  
  updateTopic();
  updatePose();
  
  emit initialized();
}

void TFMarkerDisplay::onEnable() {
  updateShowDescription();
  updateShowAxes();
  updateShowCrosshair();
  updateShowControls();
}

void TFMarkerDisplay::onDisable() {
}

void TFMarkerDisplay::reset() {
  Display::reset();
}

bool TFMarkerDisplay::transform(const geometry_msgs::PoseStamped& message, 
    geometry_msgs::PoseStamped& transformedMessage, const QString&
    targetFrame) {
  geometry_msgs::PoseStamped msg = message;
  std::string tf = targetFrame.toStdString();
  
  if (msg.header.frame_id ==
      rviz::TfFrameProperty::FIXED_FRAME_STRING.toStdString())
    msg.header.frame_id = context_->getFrameManager()->getFixedFrame();
  if (tf == rviz::TfFrameProperty::FIXED_FRAME_STRING.toStdString())
    tf = context_->getFrameManager()->getFixedFrame();
  
  try {
    tfBuffer.transform(msg, transformedMessage, tf);
  }
  catch (tf2::TransformException& exception) {
    ROS_WARN("Failed to lookup transform from [%s] to [%s]: %s",
      msg.header.frame_id.c_str(), tf.c_str(), exception.what());
    
    return false;
  }
  
  return true;
}

void TFMarkerDisplay::publish(const geometry_msgs::PoseStamped& message) {
  if (publisher)
    publisher.publish(message);
}

void TFMarkerDisplay::fromMessage(const geometry_msgs::PoseStamped& message) {
  Ogre::Vector3 position(
    message.pose.position.x,
    message.pose.position.y,
    message.pose.position.z
  );  
  Ogre::Quaternion quaternion(
    message.pose.orientation.w,
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z
  );
  Ogre::Vector3 eulerRad, eulerDeg;
  quaternionToEulerRad(quaternion, eulerRad);
  quaternionToEulerDeg(quaternion, eulerDeg);
  
  frameProperty->setValue(message.header.frame_id.c_str());  
  positionProperty->setVector(position);
  eulerRadProperty->setVector(eulerRad);
  eulerDegProperty->setVector(eulerDeg);
  quaternionProperty->setQuaternion(quaternion);
}

void TFMarkerDisplay::toMessage(geometry_msgs::PoseStamped& message, const
    ros::Time& stamp) {
  message.header.stamp = stamp;
  message.header.frame_id = frameProperty->getFrameStd();
  if (message.header.frame_id ==
      rviz::TfFrameProperty::FIXED_FRAME_STRING.toStdString())
    message.header.frame_id = context_->getFrameManager()->getFixedFrame();
  
  Ogre::Vector3 position = positionProperty->getVector();
  message.pose.position.x = position[0];
  message.pose.position.y = position[1];
  message.pose.position.z = position[2];
  
  Ogre::Quaternion orientation = quaternionProperty->getQuaternion();
  message.pose.orientation.w = orientation[0];
  message.pose.orientation.x = orientation[1];
  message.pose.orientation.y = orientation[2];
  message.pose.orientation.z = orientation[3];
}

void TFMarkerDisplay::quaternionToEulerRad(const Ogre::Quaternion& quaternion,
    Ogre::Vector3& eulerRad) {
  Ogre::Matrix3 matrix = Ogre::Matrix3::IDENTITY;
  quaternion.ToRotationMatrix(matrix);
  
  Ogre::Radian yaw, pitch, roll;
  matrix.ToEulerAnglesZYX(yaw, pitch, roll);
  
  eulerRad[0] = roll.valueRadians();
  eulerRad[1] = pitch.valueRadians();
  eulerRad[2] = yaw.valueRadians();
}

void TFMarkerDisplay::quaternionToEulerDeg(const Ogre::Quaternion& quaternion,
    Ogre::Vector3& eulerDeg) {
  Ogre::Vector3 eulerRad;
  
  quaternionToEulerRad(quaternion, eulerRad);
  eulerRadToEulerDeg(eulerRad, eulerDeg);
}

void TFMarkerDisplay::eulerRadToQuaternion(const Ogre::Vector3& eulerRad,
    Ogre::Quaternion& quaternion) {
  Ogre::Matrix3 matrix = Ogre::Matrix3::IDENTITY;
  matrix.FromEulerAnglesZYX(
    Ogre::Radian(eulerRad[2]), 
    Ogre::Radian(eulerRad[1]),
    Ogre::Radian(eulerRad[0])
  );
  
  quaternion.FromRotationMatrix(matrix);
}

void TFMarkerDisplay::eulerDegToQuaternion(const Ogre::Vector3& eulerDeg,
    Ogre::Quaternion& quaternion) {
  Ogre::Vector3 eulerRad;
  
  eulerDegToEulerRad(eulerDeg, eulerRad);
  eulerRadToQuaternion(eulerRad, quaternion);
}

void TFMarkerDisplay::eulerRadToEulerDeg(const Ogre::Vector3& eulerRad,
    Ogre::Vector3& eulerDeg) {
  eulerDeg = eulerRad*180.0/Ogre::Math::HALF_PI;
}

void TFMarkerDisplay::eulerDegToEulerRad(const Ogre::Vector3& eulerDeg,
    Ogre::Vector3& eulerRad) {
  eulerRad = eulerDeg*Ogre::Math::HALF_PI/180.0;
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void TFMarkerDisplay::updateShowDescription() {
  marker->setShowDescription(showDescriptionProperty->getBool());
}

void TFMarkerDisplay::updateDescriptionColor() {
  marker->setDescriptionColor(descriptionColorProperty->getColor());
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

void TFMarkerDisplay::updateShowControls() {
  marker->setShowControls(showControlsProperty->getBool());
  
  updateShowPositionControls();
  updateShowOrientationControls();
}

void TFMarkerDisplay::updateShowPositionControls() {
  marker->setShowPositionControls(
    showControlsProperty->getBool() &&
    showPositionControlsProperty->getBool()
  );
  
  updateShowXControls();
  updateShowYControls();
  updateShowZControls();
}

void TFMarkerDisplay::updateShowXControls() {
  marker->setShowXControls(
    showControlsProperty->getBool() &&
    showPositionControlsProperty->getBool() &&
    showXControlsProperty->getBool()
  );
}

void TFMarkerDisplay::updateShowYControls() {
  marker->setShowYControls(
    showControlsProperty->getBool() &&
    showPositionControlsProperty->getBool() &&
    showYControlsProperty->getBool()
  );
}

void TFMarkerDisplay::updateShowZControls() {
  marker->setShowZControls(
    showControlsProperty->getBool() &&
    showPositionControlsProperty->getBool() &&
    showZControlsProperty->getBool()
  );
}

void TFMarkerDisplay::updateShowOrientationControls() {
  marker->setShowOrientationControls(
    showControlsProperty->getBool() &&
    showOrientationControlsProperty->getBool()
  );

  updateShowYawControls();
  updateShowPitchControls();
  updateShowRollControls();
}

void TFMarkerDisplay::updateShowYawControls() {
  marker->setShowYawControls(
    showControlsProperty->getBool() &&
    showOrientationControlsProperty->getBool() &&
    showYawControlsProperty->getBool()
  );
}

void TFMarkerDisplay::updateShowPitchControls() {
  marker->setShowPitchControls(
    showControlsProperty->getBool() &&
    showOrientationControlsProperty->getBool() &&
    showPitchControlsProperty->getBool()
  );
}

void TFMarkerDisplay::updateShowRollControls() {
  marker->setShowRollControls(
    showControlsProperty->getBool() &&
    showOrientationControlsProperty->getBool() &&
    showRollControlsProperty->getBool()
  );
}

void TFMarkerDisplay::updatePose() {
  if (!block) {
    geometry_msgs::PoseStamped message, transformedMessage;
    
    toMessage(message);
    
    if (!message.header.frame_id.empty()) {
      if (transform(message, transformedMessage,
          rviz::TfFrameProperty::FIXED_FRAME_STRING))
        marker->setPose(transformedMessage.pose);
    }
    else
      marker->setPose(message.pose);
    
    publish(message);
  }
}

void TFMarkerDisplay::updateTopic() {
  if (publisher)
    publisher.shutdown();
  
  if (!topicProperty->getTopic().isEmpty())
    publisher = update_nh_.advertise<geometry_msgs::PoseStamped>(
      topicProperty->getTopicStd(), 1);
    
  updatePose();
}

void TFMarkerDisplay::updateFrame() {
  if (!block) {
    if (frame != frameProperty->getFrame()) {
      geometry_msgs::PoseStamped message, transformedMessage;
      
      toMessage(message);
      message.header.frame_id = frame.toStdString();
      frame = frameProperty->getFrame();
      
      if (transform(message, transformedMessage, frame)) {
        block = true;
        fromMessage(transformedMessage);
        block = false;
      }
    }
  }
  
  updatePose();
}

void TFMarkerDisplay::updatePosition() {
  updatePose();
}

void TFMarkerDisplay::updateEulerDeg() {
  if (!block) {  
    Ogre::Vector3 eulerDeg = eulerDegProperty->getVector();
    Ogre::Vector3 eulerRad;
    Ogre::Quaternion quaternion;
    
    eulerDegToEulerRad(eulerDeg, eulerRad);
    eulerRadToQuaternion(eulerRad, quaternion);

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
    Ogre::Vector3 eulerDeg;
    Ogre::Quaternion quaternion;
    
    eulerRadToEulerDeg(eulerRad, eulerDeg);
    eulerRadToQuaternion(eulerRad, quaternion);
    
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
    Ogre::Vector3 eulerRad, eulerDeg;
    
    quaternionToEulerRad(quaternion, eulerRad);
    eulerRadToEulerDeg(eulerRad, eulerDeg);
    
    block = true;
    eulerDegProperty->setVector(eulerDeg);
    eulerRadProperty->setVector(eulerRad);
    block = false;
  }
    
  updatePose();
}

}
