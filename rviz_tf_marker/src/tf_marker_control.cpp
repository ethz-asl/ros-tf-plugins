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

#include <ros/ros.h>

#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/selection/selection_handler.h>

#include "rviz_tf_marker/tf_marker.h"
#include "rviz_tf_marker/tf_marker_control.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerControl::TFMarkerControl(rviz::DisplayContext* context,
    Ogre::SceneNode* parentNode, TFMarker* parent, const QString& hint) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  parent(parent),
  dragViewport(0),
  selectionHandler(new rviz::SelectionHandler(context)),
  cursor(rviz::getDefaultCursor()),
  hint(hint),
  interactionEnabled(false),
  mouseDown(false),
  mouseDragging(false) {
  connect(parent, SIGNAL(initialized()), this, SLOT(parentInitialized()));
  connect(parent, SIGNAL(descriptionChanged(const QString&)), this,
    SLOT(parentDescriptionChanged(const QString&)));
  
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

QString TFMarkerControl::getStatus() const {
  QString description = parent->getDescription();
  
  if (!description.isEmpty())
    return description+" "+hint;
  else
    return hint;
}

void TFMarkerControl::setHighlight(double ambient) {
  for (std::set<Ogre::Pass*>::iterator it = highlightPasses.begin();
      it != highlightPasses.end(); it++)
    (*it)->setAmbient(ambient, ambient, ambient);
}

bool TFMarkerControl::isInteractive() {
  return true;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void TFMarkerControl::enableInteraction(bool enable) {
  if (!mouseDown) {
    interactionEnabled = enable;
    
    if (!enable)
      setHighlight(0.0);
  }
}

void TFMarkerControl::handleMouseEvent(rviz::ViewportMouseEvent& event) {
  if (event.type == QEvent::FocusIn) {
    setHighlight(0.3);
    context->setStatus(getStatus());
  }
  else if(event.type == QEvent::FocusOut) {
    stopDragging();
    setHighlight(0.0);
    
    return;
  }

  mouseDown = event.left() || event.middle() || event.right();
  
  if (event.leftUp()) {
    setHighlight(0.3);
    stopDragging();
  }
  else if (event.leftDown())
    setHighlight(0.5);
}

void TFMarkerControl::addMaterial(const Ogre::MaterialPtr& material) {
  Ogre::Pass* originalPass = material->getTechnique(0)->getPass(0);
  Ogre::Pass* pass = material->getTechnique(0)->createPass();

  pass->setSceneBlending(Ogre::SBT_ADD);
  pass->setDepthWriteEnabled(false);
  pass->setDepthCheckEnabled(true);
  pass->setLightingEnabled(true);
  pass->setAmbient(0.0, 0.0, 0.0);
  pass->setDiffuse(0.0, 0.0, 0.0, 0.0);
  pass->setSpecular(0.0, 0.0, 0.0, 0.0);
  pass->setCullingMode(originalPass->getCullingMode());

  materials.insert(material);
  highlightPasses.insert(pass);    
}

void TFMarkerControl::stopDragging(bool force) {
  if (mouseDragging || force) {
    mouseDragging = false;
    dragViewport = 0;
  }
}

void TFMarkerControl::orientationToColor(const Ogre::Quaternion& orientation,
    QColor& color) {
  Ogre::Matrix3 R;
  orientation.ToRotationMatrix(R);
  Ogre::Vector3 axis = R*Ogre::Vector3::UNIT_X;
  
  double x, y, z;
  x = fabs(axis[0]);
  y = fabs(axis[1]);
  z = fabs(axis[2]);

  float maxXY = x > y ? x : y;
  float maxYZ = y > z ? y : z;
  float maxXYZ = maxXY > maxYZ ? maxXY : maxYZ;

  color.setRgbF(x/maxXYZ, y/maxXYZ, z/maxXYZ, 0.5);
}

/*****************************************************************************/
/* Slots                                                                     */
/*****************************************************************************/

void TFMarkerControl::parentInitialized() {
  selectionHandler->setInteractiveObject(shared_from_this());
  selectionHandler->addTrackedObjects(sceneNode);
}

void TFMarkerControl::parentDescriptionChanged(const QString& description) {
  context->setStatus(getStatus());
}

}
