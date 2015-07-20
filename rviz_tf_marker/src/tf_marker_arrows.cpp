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
#include <rviz/load_resource.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/parse_color.h>

#include "rviz_tf_marker/tf_marker_arrows.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerArrows::TFMarkerArrows(rviz::DisplayContext* context, Ogre::SceneNode*
    parentNode, TFMarker* parent, const Ogre::Quaternion& orientation, const
    Ogre::Vector3& scale, double distance) :
  TFMarkerControl(context, parentNode, parent, true, false,
    "<b>Left-Click:</b> Move."),
  positiveArrow(new rviz::Arrow(context->getSceneManager(), sceneNode)),
  negativeArrow(new rviz::Arrow(context->getSceneManager(), sceneNode)) {
  cursor = rviz::makeIconCursor("package://rviz/icons/move1d.svg");
  
  positiveArrow->setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(
    Ogre::Vector3::UNIT_X));
  positiveArrow->set(0.5, 0.6, 0.5, 1.0);
  
  negativeArrow->setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(
    Ogre::Vector3::NEGATIVE_UNIT_X));
  negativeArrow->set(0.5, 0.6, 0.5, 1.0);
  
  setOrientation(orientation);
  setScale(scale);
  setDistance(distance);
  
  addMaterial(positiveArrow->getShaft()->getMaterial());
  addMaterial(positiveArrow->getHead()->getMaterial());
  addMaterial(negativeArrow->getShaft()->getMaterial());
  addMaterial(negativeArrow->getHead()->getMaterial());  
}

TFMarkerArrows::~TFMarkerArrows() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void TFMarkerArrows::setOrientation(const Ogre::Quaternion& orientation) {
  QColor color;
  orientationToColor(orientation, color);
  
  sceneNode->setOrientation(orientation);
  
  positiveArrow->setColor(rviz::qtToOgre(color));
  negativeArrow->setColor(rviz::qtToOgre(color));
}

void TFMarkerArrows::setScale(const Ogre::Vector3& scale) {
  positiveArrow->setScale(scale);
  negativeArrow->setScale(scale);
}

void TFMarkerArrows::setDistance(double distance) {
  positiveArrow->setPosition(Ogre::Vector3(0.5*distance, 0.0, 0.0));
  negativeArrow->setPosition(Ogre::Vector3(-0.5*distance, 0.0, 0.0));
}

}
