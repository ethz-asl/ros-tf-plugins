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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/properties/parse_color.h>

#include "rviz_tf_marker/tf_marker.h"
#include "rviz_tf_marker/tf_marker_description.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerDescription::TFMarkerDescription(rviz::DisplayContext* context,
    Ogre::SceneNode* parentNode, TFMarker* parent, const QString& text,
    double scale, const QColor& color) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  parent(parent),
  textNode(sceneNode->createChildSceneNode()),
  text(0),
  scale(1.0) {
  connect(parent, SIGNAL(poseChanged(const Ogre::Vector3&,
    const Ogre::Quaternion&)), this, SLOT(parentPoseChanged(
    const Ogre::Vector3&, const Ogre::Quaternion&)));
    
  setText(text);
  setScale(scale);
  setColor(color);
}

TFMarkerDescription::~TFMarkerDescription() {
  context->getSceneManager()->destroySceneNode(sceneNode);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

Ogre::SceneNode* TFMarkerDescription::getSceneNode() {
  return sceneNode;
}

void TFMarkerDescription::setText(const QString& text) {
  if (!this->text) {
    this->text = new rviz::MovableText(text.toStdString());
    this->text->setTextAlignment(rviz::MovableText::H_CENTER,
      rviz::MovableText::V_CENTER);
    this->text->setCharacterHeight(scale*0.15);
    setColor(color);
    
    textNode->setPosition(0, 0, scale*1.4);
    textNode->setScale(scale, scale, scale);
    textNode->attachObject(this->text);
  }
  else
    this->text->setCaption(text.toStdString());
}

QString TFMarkerDescription::getText() const {
  if (text)
    return text->getCaption().c_str();
  else
    return QString();
}

void TFMarkerDescription::setScale(double scale) {
  this->scale = scale;
  
  if (text) {
    text->setCharacterHeight(scale*0.15);
    
    textNode->setScale(scale, scale, scale);
    textNode->setPosition(0, 0, scale*1.4);
  }
}

void TFMarkerDescription::setColor(const QColor& color) {
  this->color = color;
  
  if (text)
    text->setColor(rviz::qtToOgre(color));
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void TFMarkerDescription::parentPoseChanged(const Ogre::Vector3& position,
    const Ogre::Quaternion& orientation) {
  sceneNode->setOrientation(
    parent->getSceneNode()->getOrientation().Inverse());
}

}
