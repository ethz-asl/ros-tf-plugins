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

#include "tf_marker.h"
#include "tf_marker_control.h"
#include "tf_marker_crosshair.h"
#include "tf_marker_description.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarker::TFMarker(rviz::DisplayContext* context, Ogre::SceneNode* parentNode) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  axes(new rviz::Axes(context->getSceneManager(), sceneNode, 1, 0.05)),
  description(new TFMarkerDescription(context, sceneNode)),
  crosshair(new TFMarkerCrosshair(context, sceneNode)),
  control(new TFMarkerControl(context, sceneNode, this)) {
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

void TFMarker::setShowVisualAids(bool show) {
}

void TFMarker::setPose(const Ogre::Vector3& position, const Ogre::Quaternion&
    orientation) {
  sceneNode->setPosition(position);
  
  axes->getSceneNode()->setOrientation(orientation);
  control->getSceneNode()->setOrientation(orientation);
}

void TFMarker::setPose(const geometry_msgs::PoseStamped& message) {
  Ogre::Vector3 position(
    message.pose.position.x,
    message.pose.position.y,
    message.pose.position.z
  );
  
  Ogre::Quaternion orientation(
    message.pose.orientation.w,
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z
  );
  
  // TODO: Convert to frame
  
  setPose(position, orientation);
}

}
