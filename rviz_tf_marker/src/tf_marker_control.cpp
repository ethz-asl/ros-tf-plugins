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

#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/selection/selection_manager.h>

#include "tf_marker.h"
#include "tf_marker_control.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerControl::TFMarkerControl(rviz::DisplayContext* context,
    Ogre::SceneNode* parentNode, TFMarker* parent) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  parent(parent),
  cursor(rviz::getDefaultCursor()),
  mouseDown(false),
  mouseDragging(false) {
  context->getSceneManager()->addListener(this);
}

TFMarkerControl::~TFMarkerControl() {
  context->getSceneManager()->removeListener(this);
  context->getSceneManager()->destroySceneNode(sceneNode);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

Ogre::SceneNode* TFMarkerControl::getSceneNode() {
  return sceneNode;
}

const QCursor& TFMarkerControl::getCursor() const {
  return cursor;
}

bool TFMarkerControl::isInteractive() {
  return true;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void TFMarkerControl::enableInteraction(bool enable) {
}

void TFMarkerControl::handleMouseEvent(rviz::ViewportMouseEvent& event) {
}

}
