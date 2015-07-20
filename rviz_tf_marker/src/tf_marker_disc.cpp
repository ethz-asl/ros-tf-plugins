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
#include <vector>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <ros/ros.h>

#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/properties/parse_color.h>
#include <rviz/selection/selection_handler.h>

#include "rviz_tf_marker/tf_marker_disc.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerDisc::TFMarkerDisc(rviz::DisplayContext* context, Ogre::SceneNode*
    parentNode, TFMarker* parent, const QString& name, const Ogre::Quaternion&
    orientation, const Ogre::Vector3& scale, double radius, size_t
    numSegments, bool translationEnabled) :
  TFMarkerControl(context, parentNode, parent, false, true),
  manualObject(context->getSceneManager()->createManualObject(
    name.toStdString())),
  radius(radius),
  numSegments(numSegments),
  transparent(true) {
  cursor = rviz::makeIconCursor("package://rviz/icons/moverotate.svg");
  
  sceneNode->attachObject(manualObject);
  
  setOrientation(orientation);
  setScale(scale);
  enableTranslation(translationEnabled);
}

TFMarkerDisc::~TFMarkerDisc() {
  context->getSceneManager()->destroyManualObject(manualObject);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void TFMarkerDisc::setOrientation(const Ogre::Quaternion& orientation) {
  sceneNode->setOrientation(orientation);
  updateObject(orientation, transparent);
}

void TFMarkerDisc::setScale(const Ogre::Vector3& scale) {
  sceneNode->setScale(scale);
}

void TFMarkerDisc::setTransparent(bool transparent) {
  TFMarkerControl::setTransparent(transparent);
  
  this->transparent = transparent;
  updateObject(sceneNode->getOrientation(), transparent);
}

void TFMarkerDisc::enableTranslation(bool enable) {
  translationEnabled = enable;
  
  if (enable)
    hint = "<b>Left-Click:</b> Move/Rotate.";
  else
    hint = "<b>Left-Click:</b> Rotate.";
  
  updateObject(sceneNode->getOrientation(), transparent);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void TFMarkerDisc::makeCircle(std::vector<Ogre::Vector3>& vertices,
    double radius, size_t numSegments) {
  vertices.resize(numSegments);
  
  for (size_t i = 0; i < numSegments; ++i) {
    double theta = double(i)/double(numSegments)*Ogre::Math::TWO_PI;

    vertices[i][0] = 0.0;
    vertices[i][1] = radius*cos(theta);
    vertices[i][2] = radius*sin(theta);
  }
}

void TFMarkerDisc::makeDisc(std::vector<Ogre::Vector3>& vertices,
    std::vector<Ogre::ColourValue>& colors, double innerRadius, double
    outerRadius, size_t numSegments, const QColor& color) {
  vertices.resize(6*numSegments);
    
  std::vector<Ogre::Vector3> innerCircle, outerCircle;
  makeCircle(innerCircle, innerRadius, numSegments);
  makeCircle(outerCircle, outerRadius, numSegments);
  
  if (translationEnabled && rotationEnabled) {
    colors.resize(2*numSegments);
    
    for (size_t i = 0; i < numSegments-1; i += 2) {
      size_t i1 = i;
      size_t i2 = (i+1)%numSegments;
      size_t i3 = (i+2)%numSegments;

      size_t p = i*6;
      size_t c = i*2;

      vertices[p+0] = innerCircle[i1];
      vertices[p+1] = outerCircle[i2];
      vertices[p+2] = innerCircle[i2];

      vertices[p+3] = innerCircle[i2];
      vertices[p+4] = outerCircle[i2];
      vertices[p+5] = innerCircle[i3];

      colors[c] = rviz::qtToOgre(color)*0.6;
      colors[c].a = color.alphaF();
      colors[c+1] = colors[c];

      p += 6;
      c += 2;

      vertices[p+0] = outerCircle[i1];
      vertices[p+1] = outerCircle[i2];
      vertices[p+2] = innerCircle[i1];

      vertices[p+3] = outerCircle[i2];
      vertices[p+4] = outerCircle[i3];
      vertices[p+5] = innerCircle[i3];

      colors[c] = rviz::qtToOgre(color);
      colors[c+1] = colors[c];
    }
  }
  else if (!translationEnabled && rotationEnabled) {
    colors.resize(2*numSegments);

    for (size_t i = 0; i < numSegments; ++i) {
      size_t i1 = i;
      size_t i2 = (i+1)%numSegments;
      size_t i3 = (i+2)%numSegments;

      size_t p = i*6;
      size_t c = i*2;

      vertices[p+0] = innerCircle[i1];
      vertices[p+1] = outerCircle[i2];
      vertices[p+2] = innerCircle[i2];

      vertices[p+3] = innerCircle[i2];
      vertices[p+4] = outerCircle[i2];
      vertices[p+5] = outerCircle[i3];

      double t = 0.6+0.4*(i%2);
      colors[c] = rviz::qtToOgre(color)*t;
      colors[c].a = color.alphaF();
      colors[c+1] = colors[c];
    }
  }
  else if (translationEnabled && !rotationEnabled) {
    for (size_t i = 0; i < numSegments; ++i) {
      size_t i1 = i;
      size_t i2 = (i+1)%numSegments;

      size_t p = i*6;

      vertices[p+0] = innerCircle[i1];
      vertices[p+1] = outerCircle[i1];
      vertices[p+2] = innerCircle[i2];

      vertices[p+3] = outerCircle[i1];
      vertices[p+4] = outerCircle[i2];
      vertices[p+5] = innerCircle[i2];
    }
  }
}

void TFMarkerDisc::updateObject(const Ogre::Quaternion& orientation, bool
    transparent) {
  QColor color;
  orientationToColor(orientation, color);
  
  std::vector<Ogre::Vector3> vertices;
  std::vector<Ogre::ColourValue> colors;
  
  makeDisc(vertices, colors, 0.75*radius, radius, numSegments, color);
  
  if (material.isNull()) {
    static size_t count = 0;
    std::stringstream stream;
    stream << "disc_material_" << count++;
    
    material = Ogre::MaterialManager::getSingleton().create(stream.str(),
      "rviz_tf_marker");
    
    material->setReceiveShadows(false);
    material->setCullingMode(Ogre::CULL_NONE);
    
    addMaterial(material);
  }
  
  if (transparent && (color.alphaF() < 0.9998)) {
    material->getTechnique(0)->getPass(0)->setSceneBlending(
      Ogre::SBT_TRANSPARENT_ALPHA);
    material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  }
  else {
    material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
  }
  
  if (colors.empty()) {
    Ogre::ColourValue colour = rviz::qtToOgre(color);
    
    material->getTechnique(0)->getPass(0)->setAmbient(colour*0.5);
    material->getTechnique(0)->getPass(0)->setDiffuse(colour);
    
    material->getTechnique(0)->getPass(0)->setLightingEnabled(true);
  }
  else
    material->getTechnique(0)->getPass(0)->setLightingEnabled(false);


  selectionHandler->removeTrackedObject(manualObject);
  
  manualObject->clear();
  manualObject->estimateVertexCount(vertices.size());
  manualObject->begin(material->getName(),
    Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  for(size_t i = 0; i < vertices.size(); i += 3) {
    Ogre::Vector3 normal = (vertices[i+1]-vertices[i]).crossProduct(
      vertices[i+2]-vertices[i]);
    normal.normalise();
    
    for (size_t j = 0; j < 3; ++j) {
      manualObject->position(vertices[i+j]);
      manualObject->normal(normal);
      
      if (!colors.empty()) {
        manualObject->colour(
          colors[i/3].r,
          colors[i/3].g,
          colors[i/3].b,
          transparent ? colors[i/3].a : 1.0
        );
      }
    }
  }

  manualObject->end();  
  
  selectionHandler->addTrackedObject(manualObject);
}

}
