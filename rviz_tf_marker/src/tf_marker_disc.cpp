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

#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/properties/parse_color.h>

#include "rviz_tf_marker/tf_marker_disc.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerDisc::TFMarkerDisc(rviz::DisplayContext* context, Ogre::SceneNode*
    parentNode, TFMarker* parent, const QString& name, const Ogre::Quaternion&
    orientation, const Ogre::Vector3& scale, double radius, size_t
    numSegments, const QString& hint) :
  TFMarkerControl(context, parentNode, parent, hint),
  manualObject(context->getSceneManager()->createManualObject(
    name.toStdString())),
  radius(radius),
  numSegments(numSegments) {
  cursor = rviz::makeIconCursor("package://rviz/icons/moverotate.svg");
  
  sceneNode->attachObject(manualObject);
  
  setOrientation(orientation);
  setScale(scale);
}

TFMarkerDisc::~TFMarkerDisc() {
  context->getSceneManager()->destroyManualObject(manualObject);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void TFMarkerDisc::setOrientation(const Ogre::Quaternion& orientation) {
  sceneNode->setOrientation(orientation);
  updateObject(orientation);
}

void TFMarkerDisc::setScale(const Ogre::Vector3& scale) {
  sceneNode->setScale(scale);
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
    std::vector<Ogre::ColourValue>& colors, const Ogre::Quaternion&
    orientation, double innerRadius, double outerRadius, size_t
    numSegments) {
  vertices.resize(6*numSegments);
  colors.resize(2*numSegments);
    
  std::vector<Ogre::Vector3> innerCircle, outerCircle;
  makeCircle(innerCircle, innerRadius, numSegments);
  makeCircle(outerCircle, outerRadius, numSegments);
  
  QColor color;
  orientationToColor(orientation, color);
  
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

void TFMarkerDisc::updateObject(const Ogre::Quaternion& orientation) {
  std::vector<Ogre::Vector3> vertices;
  std::vector<Ogre::ColourValue> colors;
  
  makeDisc(vertices, colors, orientation, 0.75*radius, radius, numSegments);
  
  if (materials.empty()) {
    static size_t count = 0;
    std::stringstream stream;
    stream << "disc_material_" << count++;
    
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
      stream.str(), "rviz_tf_marker");
    
    material->setReceiveShadows(false);
    material->setCullingMode(Ogre::CULL_NONE);
    material->getTechnique(0)->setLightingEnabled(false);
    material->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->getTechnique(0)->setDepthWriteEnabled(false);
    
    addMaterial(material);
  }
  
  manualObject->clear();
  manualObject->estimateVertexCount(vertices.size());
  manualObject->begin((*materials.begin())->getName(),
    Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  for(size_t i = 0; i < vertices.size(); i += 3) {
    Ogre::Vector3 normal = (vertices[i+1]-vertices[i]).crossProduct(
      vertices[i+2]-vertices[i]);
    normal.normalise();
    
    for (size_t j = 0; j < 3; ++j) {
      manualObject->position(vertices[i+j]);
      manualObject->normal(normal);
      manualObject->colour(colors[i/3]);
    }
  }

  manualObject->end();  
}

}
