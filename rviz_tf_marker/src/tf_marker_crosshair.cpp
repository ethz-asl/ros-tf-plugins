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

#include <sstream>

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include <ros/ros.h>

#include <rviz/display_context.h>
#include <rviz/mesh_loader.h>
#include <rviz/properties/parse_color.h>

#include "rviz_tf_marker/tf_marker_crosshair.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerCrosshair::TFMarkerCrosshair(rviz::DisplayContext* context,
    Ogre::SceneNode* parentNode, const QString& resource, double scale,
    const QColor& color) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  entity(0),
  scale(1.0) {
  setResource(resource);
  setScale(scale);
  setColor(color);
  
  context->getSceneManager()->addListener(this);
}

TFMarkerCrosshair::~TFMarkerCrosshair() {
  context->getSceneManager()->removeListener(this);
  context->getSceneManager()->destroySceneNode(sceneNode);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

Ogre::SceneNode* TFMarkerCrosshair::getSceneNode() {
  return sceneNode;
}

void TFMarkerCrosshair::setResource(const QString& resource) {
  if (entity) {
    context->getSceneManager()->destroyEntity(entity);
    entity = 0;
  }
  
  if (!resource.isEmpty()) {
    if (!rviz::loadMeshFromResource(resource.toStdString()).isNull()) {
      entity = context->getSceneManager()->createEntity("crosshair",
        resource.toStdString());
      
      setColor(color);
      sceneNode->attachObject(entity);
    }
    else
      ROS_WARN("Failed to load mesh resource from [%s].",
        resource.toStdString().c_str());
  }
}

void TFMarkerCrosshair::setScale(double scale) {
  sceneNode->setScale(scale, scale, scale);
}

void TFMarkerCrosshair::setColor(const QColor& color) {
  this->color = color;
  
  if (entity) {
    if (material.isNull()) {
      static size_t count = 0;
      std::stringstream stream;
      stream << "crosshair_material_" << count++;
      
      material = Ogre::MaterialManager::getSingleton().create(
        stream.str(), "rviz_tf_marker");
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(true);
      
      entity->setMaterialName(material->getName());
    }
    
    Ogre::ColourValue colour = rviz::qtToOgre(color);
    
    material->getTechnique(0)->setAmbient(colour*0.5);
    material->getTechnique(0)->setDiffuse(colour);

    if (colour.a < 0.9998) {
      material->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      material->getTechnique(0)->setDepthWriteEnabled(false);
    }
    else {
      material->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
      material->getTechnique(0)->setDepthWriteEnabled(true);
    }
  }
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void TFMarkerCrosshair::preFindVisibleObjects(Ogre::SceneManager* source,
    Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport* v) {
  Ogre::Quaternion xViewFacingRotation = Ogre::Quaternion::IDENTITY.
    xAxis().getRotationTo(v->getCamera()->getDerivedDirection());
  Ogre::Vector3 zAxis2 = xViewFacingRotation*
    Ogre::Quaternion::IDENTITY.zAxis();
  Ogre::Quaternion alignYZRotation = zAxis2.getRotationTo(
    v->getCamera()->getDerivedUp());
  Ogre::Quaternion rotateAroundX = Ogre::Quaternion(Ogre::Radian(0.0),
    v->getCamera()->getDerivedDirection());
  Ogre::Quaternion rotation = sceneNode->getParentSceneNode()->
    convertWorldToLocalOrientation(rotateAroundX*alignYZRotation*
    xViewFacingRotation);

  sceneNode->setOrientation(rotation);
  sceneNode->_update(true, false);
}

}
