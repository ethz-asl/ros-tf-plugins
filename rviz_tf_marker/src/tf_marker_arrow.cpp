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

#include "rviz_tf_marker/tf_marker_arrow.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerArrow::TFMarkerArrow(rviz::DisplayContext* context, Ogre::SceneNode*
    parentNode, TFMarker* parent, const Ogre::Quaternion& orientation, const
    Ogre::Vector3& scale, double offset, const QString& hint) :
  TFMarkerControl(context, parentNode, parent, hint),
  arrow(new rviz::Arrow(context->getSceneManager(), sceneNode)) {
  cursor = rviz::makeIconCursor("package://rviz/icons/move1d.svg");
  
  arrow->setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(
    Ogre::Vector3::UNIT_X));
  arrow->set(0.5, 0.5, 0.5, 1.0);
  
  addMaterial(arrow->getShaft()->getMaterial());
  addMaterial(arrow->getHead()->getMaterial());
  
  setOrientation(orientation);
  setScale(scale);
  setOffset(offset);
}

TFMarkerArrow::~TFMarkerArrow() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void TFMarkerArrow::setOrientation(const Ogre::Quaternion& orientation) {
  QColor color;
  orientationToColor(orientation, color);
  
  sceneNode->setOrientation(orientation);
  arrow->setColor(rviz::qtToOgre(color));
}

void TFMarkerArrow::setScale(const Ogre::Vector3& scale) {
  sceneNode->setScale(arrow->getOrientation().Inverse()*scale);
}

void TFMarkerArrow::setOffset(double offset) {
  sceneNode->setPosition(sceneNode->getOrientation()*
    Ogre::Vector3(offset, 0.0, 0.0));
}

}
