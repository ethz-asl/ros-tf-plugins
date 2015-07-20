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

#include <limits>

#include <OgreSceneNode.h>

#include <ros/ros.h>

#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/selection/selection_handler.h>
#include <rviz/selection/selection_manager.h>

#include "rviz_tf_marker/tf_marker.h"
#include "rviz_tf_marker/tf_marker_control.h"

namespace rviz_tf_marker {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

TFMarkerControl::TFMarkerControl(rviz::DisplayContext* context,
    Ogre::SceneNode* parentNode, TFMarker* parent, bool translationEnabled,
    bool rotationEnabled, const QString& hint) :
  context(context),
  sceneNode(parentNode->createChildSceneNode()),
  parent(parent),
  selectionHandler(new rviz::SelectionHandler(context)),
  line(new rviz::Line(context->getSceneManager(), sceneNode)),
  cursor(rviz::getDefaultCursor()),
  hint(hint),
  interactionEnabled(false),
  visible(true),
  translationEnabled(translationEnabled),
  rotationEnabled(rotationEnabled),
  showVisualAids(false),
  mouseDown(false),
  mouseDragging(false) {
  connect(parent, SIGNAL(initialized()), this, SLOT(parentInitialized()));
  connect(parent, SIGNAL(descriptionChanged(const QString&)), this,
    SLOT(parentDescriptionChanged(const QString&)));
  connect(parent, SIGNAL(visualAidsShown(bool)), this,
    SLOT(parentVisualAidsShown(bool)));
  connect(parent, SIGNAL(transparencyEnabled(bool)), this,
    SLOT(parentTransparencyEnabled(bool)));
  
  context->getSceneManager()->addListener(this);
  
  Ogre::Vector3 direction = Ogre::Quaternion::IDENTITY.xAxis()*10000.0;
  line->setPoints(direction, -1.0*direction);
  line->setVisible(false);
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

void TFMarkerControl::setVisible(bool visible) {
  this->visible = visible;
  
  sceneNode->setVisible(interactionEnabled && visible);
  line->setVisible(visible && showVisualAids && mouseDragging);
}

void TFMarkerControl::setShowVisualAids(bool show) {
  showVisualAids = show;
  
  setVisible(visible);
}

void TFMarkerControl::setTransparent(bool transparent) {
  for (std::set<Ogre::MaterialPtr>::iterator it = materials.begin();
      it != materials.end(); ++it) {
    if (transparent) {
      (*it)->getTechnique(0)->getPass(0)->setSceneBlending(
        Ogre::SBT_TRANSPARENT_ALPHA);
      (*it)->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    }
    else {
      (*it)->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
      (*it)->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
    }
  }
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
    
    setVisible(visible);
    
    if (!enable)
      setHighlight(0.0);
  }
}

void TFMarkerControl::enableTranslation(bool enable) {
  translationEnabled = enable;
}

void TFMarkerControl::enableRotation(bool enable) {
  rotationEnabled = enable;
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
  else if (event.leftDown()) {
    setHighlight(0.5);
    startDragging(event);
  }

  if ((event.type == QEvent::MouseMove) && event.left() && mouseDragging &&
      ((event.x != event.last_x) || (event.y != event.last_y)))
    drag(event);
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

void TFMarkerControl::startDragging(rviz::ViewportMouseEvent& event) {  
  Ogre::Vector3 mousePosition;
  
  if (context->getSelectionManager()->get3DPoint(event.viewport,
      event.x, event.y, mousePosition)) {
    parent->startDragging();
    mouseDragging = true;
  
    grabPosition = mousePosition;
  
    rotationAxis = parent->getReferenceNode()->
      convertLocalToWorldPosition(sceneNode->getOrientation()*
      Ogre::Vector3::UNIT_X)-parent->getReferenceNode()->
      convertLocalToWorldPosition(Ogre::Vector3(0.0, 0.0, 0.0));
    rotationAxis.normalise();
    rotationCenter = closestPointOnLineToPoint(
      parent->getReferenceNode()->convertLocalToWorldPosition(
      Ogre::Vector3(0.0, 0.0, 0.0)), rotationAxis, grabPosition);
    
    setVisible(visible);
  }
}

void TFMarkerControl::drag(rviz::ViewportMouseEvent& event) {
  Ogre::Ray mouseRay = mouseToRay(event, event.x, event.y);

  if (translationEnabled && !rotationEnabled)
    translate(mouseRay, event);
  else if (!translationEnabled && rotationEnabled)
    rotate(mouseRay, event);
  else if (translationEnabled && rotationEnabled)
    translateRotate(mouseRay, event);
}

void TFMarkerControl::stopDragging(bool force) {
  if (mouseDragging || force) {
    mouseDragging = false;
    parent->stopDragging();
    
    setVisible(visible);
  }
}

void TFMarkerControl::translate(const Ogre::Ray& mouseRay, const
    rviz::ViewportMouseEvent& event) {
  Ogre::Ray controlRay;
  controlRay.setOrigin(grabPosition);
  controlRay.setDirection(parent->getReferenceNode()->
    convertLocalToWorldOrientation(sceneNode->getOrientation())*
    Ogre::Vector3::UNIT_X);
  
  Ogre::Vector2 controlRayScreenStart, controlRayScreenEnd;
  worldToScreen(controlRay.getOrigin(), event.viewport, controlRayScreenStart);
  worldToScreen(controlRay.getPoint(1), event.viewport, controlRayScreenEnd);

  Ogre::Vector2 mousePosition(event.x, event.y);
  Ogre::Vector2 controlRayScreenDirection = controlRayScreenEnd-
    controlRayScreenStart;
  double denominator = controlRayScreenDirection.dotProduct(
    controlRayScreenDirection);
  
  if (fabs(denominator) > Ogre::Matrix3::EPSILON) {
    double factor = (mousePosition-controlRayScreenStart).dotProduct(
      controlRayScreenDirection)/denominator;
    
    Ogre::Vector2 closestScreenPoint = controlRayScreenStart+
      controlRayScreenDirection*factor;
    Ogre::Ray newMouseRay = mouseToRay(event, closestScreenPoint.x,
      closestScreenPoint.y);

    Ogre::Vector3 closestPoint;
    if (findClosestPoint(controlRay, newMouseRay, closestPoint)) {
      parent->setPose(
        parent->getReferenceNode()->getParent()->
          convertWorldToLocalPosition(closestPoint)-
        parent->getReferenceNode()->getParent()->
          convertWorldToLocalPosition(grabPosition)+
        parent->getReferenceNode()->getPosition(),
        parent->getSceneNode()->getOrientation()
      );
    }
  }
}

void TFMarkerControl::rotate(const Ogre::Ray& mouseRay, const
    rviz::ViewportMouseEvent& event) {
  Ogre::Vector3 previousDragPosition = grabPosition;
  Ogre::Vector3 newDragPosition;
  Ogre::Vector2 intersection2D;
  double rayLength;
  
  if (intersectSomeYZPlane(mouseRay, rotationCenter,
      sceneNode->getParent()->convertLocalToWorldOrientation(
      sceneNode->getOrientation()), newDragPosition, intersection2D,
      rayLength)) {
    Ogre::Vector3 previousCenter = previousDragPosition-rotationCenter;
    Ogre::Vector3 newCenter = newDragPosition-rotationCenter;
  
    if (newCenter.length() > Ogre::Matrix3::EPSILON) {
      Ogre::Quaternion relativeRotation = previousCenter.getRotationTo(
        newCenter, rotationAxis);
      Ogre::Radian angle, relativeAngle;
      Ogre::Vector3 axis;
      
      relativeRotation.ToAngleAxis(angle, axis);
      
      parent->setPose(
        parent->getSceneNode()->getPosition(),
        parent->getSceneNode()->getParent()->convertWorldToLocalOrientation(
          relativeRotation*parent->getReferenceNode()->getParent()->
          convertLocalToWorldOrientation(
          parent->getReferenceNode()->getOrientation()))
      );
    }
  }
}

void TFMarkerControl::translateRotate(const Ogre::Ray& mouseRay, const
    rviz::ViewportMouseEvent& event) {
  Ogre::Vector3 previousDragPosition = grabPosition;
  Ogre::Vector3 newDragPosition;
  Ogre::Vector2 intersection2D;
  double rayLength;
  
  if (intersectSomeYZPlane(mouseRay, rotationCenter,
      sceneNode->getParent()->convertLocalToWorldOrientation(
      sceneNode->getOrientation()), newDragPosition, intersection2D,
      rayLength)) {
    Ogre::Vector3 previousCenter = previousDragPosition-rotationCenter;
    Ogre::Vector3 newCenter = newDragPosition-rotationCenter;
  
    if (newCenter.length() > Ogre::Matrix3::EPSILON) {
      Ogre::Vector3 translation = newCenter*
        (1.0-previousCenter.length()/newCenter.length());
      Ogre::Quaternion rotation = previousCenter.getRotationTo(
        newCenter, rotationAxis);
      
      parent->setPose(
        parent->getSceneNode()->getParent()->convertWorldToLocalPosition(
          parent->getReferenceNode()->getParent()->
          convertLocalToWorldPosition(
          parent->getReferenceNode()->getPosition())+translation),
        parent->getSceneNode()->getParent()->convertWorldToLocalOrientation(
          rotation*parent->getReferenceNode()->getParent()->
          convertLocalToWorldOrientation(
          parent->getReferenceNode()->getOrientation()))
      );
    }
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

  double maxXY = x > y ? x : y;
  double maxYZ = y > z ? y : z;
  double maxXYZ = maxXY > maxYZ ? maxXY : maxYZ;

  color.setRgbF(x/maxXYZ, y/maxXYZ, z/maxXYZ, 0.5);
}

void TFMarkerControl::worldToScreen(const Ogre::Vector3& position,
    const Ogre::Viewport* viewport, Ogre::Vector2& screenPosition) {
  const Ogre::Camera* camera = viewport->getCamera();
  Ogre::Vector3 homogeneousScreenPosition = camera->getProjectionMatrix()*
    (camera->getViewMatrix()*position);

  double halfWidth = 0.5*viewport->getActualWidth();
  double halfHeight = 0.5*viewport->getActualHeight();

  screenPosition.x = halfWidth+(halfWidth*homogeneousScreenPosition.x)-0.5;
  screenPosition.y = halfHeight+(halfHeight*-homogeneousScreenPosition.y)-0.5;
}

bool TFMarkerControl::findClosestPoint(const Ogre::Ray& targetRay, const
    Ogre::Ray& mouseRay, Ogre::Vector3& closestPoint) {
  Ogre::Vector3 v13 = targetRay.getOrigin()-mouseRay.getOrigin();
  Ogre::Vector3 v43 = mouseRay.getDirection();
  Ogre::Vector3 v21 = targetRay.getDirection();
  
  double d1343 = v13.dotProduct(v43);
  double d4321 = v43.dotProduct(v21);
  double d1321 = v13.dotProduct(v21);
  double d4343 = v43.dotProduct(v43);
  double d2121 = v21.dotProduct(v21);

  double denominator = d2121*d4343-d4321*d4321;
  if (fabs(denominator) > Ogre::Matrix3::EPSILON) {
    double enumerator = d1343*d4321-d1321*d4343;
    double mua = enumerator/denominator;
    
    closestPoint = targetRay.getPoint(mua);
    
    return true;
  }
  
  return false;
}

Ogre::Vector3 TFMarkerControl::closestPointOnLineToPoint(const Ogre::Vector3&
    lineStart, const Ogre::Vector3& lineDirection, const Ogre::Vector3&
    testPoint) {
  double factor = (testPoint-lineStart).dotProduct(lineDirection)/
    lineDirection.dotProduct(lineDirection);
  Ogre::Vector3 closestPoint = lineStart+lineDirection*factor;
  
  return closestPoint;
}

Ogre::Ray TFMarkerControl::mouseToRay(const rviz::ViewportMouseEvent& event,
    int x, int y) {
  double width = event.viewport->getActualWidth()-1;
  double height = event.viewport->getActualHeight()-1;

  Ogre::Ray mouseRay = event.viewport->getCamera()->getCameraToViewportRay(
    (x+0.5)/width, (y+0.5)/height);

  mouseRay.setOrigin(mouseRay.getOrigin());
  mouseRay.setDirection(mouseRay.getDirection());

  return mouseRay;
}

bool TFMarkerControl::intersectSomeYZPlane(const Ogre::Ray& mouseRay, const
    Ogre::Vector3& pointInPlane, const Ogre::Quaternion& planeOrientation,
    Ogre::Vector3& intersection3D, Ogre::Vector2& intersection2D, double&
    rayLength) {
  Ogre::Vector3 normal = planeOrientation*Ogre::Vector3::UNIT_X;
  Ogre::Vector3 axis1 = planeOrientation*Ogre::Vector3::UNIT_Y;
  Ogre::Vector3 axis2 = planeOrientation*Ogre::Vector3::UNIT_Z;

  Ogre::Plane plane(normal, pointInPlane);
  Ogre::Vector2 origin2D(pointInPlane.dotProduct(axis1),
    pointInPlane.dotProduct(axis2));

  std::pair<bool, Ogre::Real> intersection = mouseRay.intersects(plane);
  if (intersection.first) {
    intersection3D = mouseRay.getPoint(intersection.second);
    intersection2D = Ogre::Vector2(intersection3D.dotProduct(axis1),
      intersection3D.dotProduct(axis2));
    intersection2D -= origin2D;
    rayLength = intersection.second;
    
    return true;
  }
  else
    rayLength = 0.0;
  
  return false;
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

void TFMarkerControl::parentVisualAidsShown(bool shown) {
  setShowVisualAids(shown);
}

void TFMarkerControl::parentTransparencyEnabled(bool enabled) {
  setTransparent(enabled);
}

}
