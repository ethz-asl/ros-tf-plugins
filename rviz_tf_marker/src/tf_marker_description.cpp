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

#include "tf_marker_description.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerDescription::TFMarkerDescription(rviz::DisplayContext* context,
    Ogre::SceneNode* parentNode, double scale) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  text(0),
  scale(1.0) {
  setScale(scale);
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

void TFMarkerDescription::setDescription(const QString& description) {
  if (!text) {
    text = new rviz::MovableText(description.toStdString());
    text->setTextAlignment(rviz::MovableText::H_CENTER,
      rviz::MovableText::V_CENTER);
    text->setCharacterHeight(scale*0.15);
    text->setColor(Ogre::ColourValue(1.0, 1.0, 1.0, 1.0));
    
    sceneNode->setPosition(0, 0, scale*1.4);
    sceneNode->setScale(scale, scale, scale);
    sceneNode->attachObject(text);
  }
  
  text->setCaption(description.toStdString());
}

void TFMarkerDescription::setScale(double scale) {
  this->scale = scale;
  
  if (text) {
    text->setCharacterHeight(scale*0.15);
    
    sceneNode->setScale(scale, scale, scale);
    sceneNode->setPosition(0, 0, scale*1.4);
  }
}

}
