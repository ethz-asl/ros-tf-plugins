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

#include <OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/selection/selection_manager.h>

#include "rviz_tf_marker/tf_marker.h"
#include "rviz_tf_marker/tf_marker_arrow.h"
#include "rviz_tf_marker/tf_marker_crosshair.h"
#include "rviz_tf_marker/tf_marker_description.h"
#include "rviz_tf_marker/tf_marker_disc.h"
#include "rviz_tf_marker/tf_marker_display.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarker::TFMarker(rviz::DisplayContext* context, Ogre::SceneNode*
    parentNode, TFMarkerDisplay* parent) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  controlsNode(sceneNode->createChildSceneNode()),
  positionControlsNode(controlsNode->createChildSceneNode()),
  orientationControlsNode(controlsNode->createChildSceneNode()),
  parent(parent),
  axes(new rviz::Axes(context->getSceneManager(), sceneNode, 1, 0.05)),
  description(new TFMarkerDescription(context, sceneNode)),
  crosshair(new TFMarkerCrosshair(context, sceneNode)),
  positiveXControl(new TFMarkerArrow(context, positionControlsNode, this,
    Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::UNIT_X))),
  negativeXControl(new TFMarkerArrow(context, positionControlsNode, this,
    Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::NEGATIVE_UNIT_X))),
  positiveYControl(new TFMarkerArrow(context, positionControlsNode, this,
    Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::UNIT_Y))),
  negativeYControl(new TFMarkerArrow(context, positionControlsNode, this,
    Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::NEGATIVE_UNIT_Y))),
  positiveZControl(new TFMarkerArrow(context, positionControlsNode, this,
    Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::UNIT_Z))),
  negativeZControl(new TFMarkerArrow(context, positionControlsNode, this,
    Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::NEGATIVE_UNIT_Z))),
   yawControl(new TFMarkerDisc(context, orientationControlsNode, this,
    "disc_yaw", Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::UNIT_Z))),
   pitchControl(new TFMarkerDisc(context, orientationControlsNode, this,
    "disc_pitch", Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::UNIT_Y))),
   rollControl(new TFMarkerDisc(context, orientationControlsNode, this,
    "disc_roll", Ogre::Vector3::UNIT_X.getRotationTo(Ogre::Vector3::UNIT_X))) {
  connect(parent, SIGNAL(initialized()), this, SLOT(parentInitialized()));
     
  crosshair->setScale(0.75);
}

TFMarker::~TFMarker() {
  delete axes;
  context->getSceneManager()->destroySceneNode(sceneNode);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void TFMarker::setDescription(const QString& description) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  this->description->setDescription(description);

  emit descriptionChanged(description);
}

QString TFMarker::getDescription() const {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  return description->getDescription();
}

void TFMarker::setDescriptionColor(const QColor& color) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  description->setColor(color);
}

void TFMarker::setShowDescription(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  description->getSceneNode()->setVisible(show);
}

void TFMarker::setShowAxes(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  axes->getSceneNode()->setVisible(show);
}

void TFMarker::setShowCrosshair(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  crosshair->getSceneNode()->setVisible(show);
}

void TFMarker::setCrosshairColor(const QColor& color) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  crosshair->setColor(color);
}

void TFMarker::setShowControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  controlsNode->setVisible(show);
}

void TFMarker::setShowPositionControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  positionControlsNode->setVisible(show);
}

void TFMarker::setShowXControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  positiveXControl->getSceneNode()->setVisible(show);
  negativeXControl->getSceneNode()->setVisible(show);
}

void TFMarker::setShowYControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  positiveYControl->getSceneNode()->setVisible(show);
  negativeYControl->getSceneNode()->setVisible(show);
}

void TFMarker::setShowZControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  positiveZControl->getSceneNode()->setVisible(show);
  negativeZControl->getSceneNode()->setVisible(show);
}

void TFMarker::setShowOrientationControls(bool show) {
  orientationControlsNode->setVisible(show);
}

void TFMarker::setShowYawControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  yawControl->getSceneNode()->setVisible(show);
}

void TFMarker::setShowPitchControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  pitchControl->getSceneNode()->setVisible(show);
}

void TFMarker::setShowRollControls(bool show) {
  boost::recursive_mutex::scoped_lock lock(mutex);
  
  rollControl->getSceneNode()->setVisible(show);
}

void TFMarker::setPose(const Ogre::Vector3& position, const Ogre::Quaternion&
    orientation) {
  sceneNode->setPosition(position);
  
  axes->getSceneNode()->setOrientation(orientation);
}

void TFMarker::setPose(const geometry_msgs::Pose& message) {
  Ogre::Vector3 position(
    message.position.x,
    message.position.y,
    message.position.z
  );
  
  Ogre::Quaternion orientation(
    message.orientation.w,
    message.orientation.x,
    message.orientation.y,
    message.orientation.z
  );
  
  setPose(position, orientation);
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void TFMarker::parentInitialized() {
  emit initialized();
}

}
