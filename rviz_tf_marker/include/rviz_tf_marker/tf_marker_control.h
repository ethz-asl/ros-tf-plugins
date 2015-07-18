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

#ifndef TF_MARKER_CONTROL_H
#define TF_MARKER_CONTROL_H

#ifndef Q_MOC_RUN
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>

#include <rviz/interactive_object.h>
#include <rviz/viewport_mouse_event.h>
#endif

#include <QColor>
#include <QCursor>
#include <QObject>

namespace Ogre {
  class Pass;
  class SceneNode;
};

namespace rviz {
  class DisplayContext;
  class Line;
  class SelectionHandler;
};

namespace rviz_tf_marker {
  class TFMarker;
    
  class TFMarkerControl :
    public QObject,
    public Ogre::SceneManager::Listener,
    public rviz::InteractiveObject,
    public boost::enable_shared_from_this<TFMarkerControl> {
  Q_OBJECT
  public:
    TFMarkerControl(rviz::DisplayContext* context, Ogre::SceneNode*
      parentNode, TFMarker* parent, bool translationEnabled = true,
      bool rotationEnabled = true, const QString& hint = QString());
    virtual ~TFMarkerControl();
    
    Ogre::SceneNode* getSceneNode();
    const QCursor& getCursor() const;
    QString getStatus() const;
    void setVisible(bool visible);
    void setShowVisualAids(bool show);
    virtual void setTransparent(bool transparent);
    void setHighlight(double ambient);
    bool isInteractive();

    void enableInteraction(bool enable);
    void enableTranslation(bool enable);
    void enableRotation(bool enable);
    void handleMouseEvent(rviz::ViewportMouseEvent& event);
  
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    TFMarker* parent;
    
    boost::shared_ptr<rviz::SelectionHandler> selectionHandler;
    boost::shared_ptr<rviz::Line> line;
  
    std::set<Ogre::MaterialPtr> materials;
    std::set<Ogre::Pass*> highlightPasses;

    Ogre::Vector3 grabPosition;
    Ogre::Vector3 rotationCenter;
    Ogre::Vector3 rotationAxis;
    
    QCursor cursor;
    QString hint;
    
    bool interactionEnabled;
    bool visible;
    bool translationEnabled;
    bool rotationEnabled;
    bool showVisualAids;
    
    bool mouseDown;
    bool mouseDragging;
    
    void addMaterial(const Ogre::MaterialPtr& material);
    
    void startDragging(rviz::ViewportMouseEvent& event);
    void drag(rviz::ViewportMouseEvent& event);
    void stopDragging(bool force = false);
    
    void translate(const Ogre::Ray& mouseRay, const
      rviz::ViewportMouseEvent& event);
    void rotate(const Ogre::Ray& mouseRay, const
      rviz::ViewportMouseEvent& event);
    void translateRotate(const Ogre::Ray& mouseRay, const
      rviz::ViewportMouseEvent& event);
    
    void orientationToColor(const Ogre::Quaternion& orientation,
      QColor& color);
    void worldToScreen(const Ogre::Vector3& position, const Ogre::Viewport*
      viewport, Ogre::Vector2& screenPosition);
    bool findClosestPoint(const Ogre::Ray& targetRay, const Ogre::Ray&
      mouseRay, Ogre::Vector3& closestPoint);
    Ogre::Vector3 closestPointOnLineToPoint(const Ogre::Vector3& lineStart,
      const Ogre::Vector3& lineDirection, const Ogre::Vector3& testPoint);
    Ogre::Ray mouseToRay(const rviz::ViewportMouseEvent& event, int x, int y);
    bool intersectSomeYZPlane(const Ogre::Ray& mouseRay, const Ogre::Vector3&
      pointInPlane, const Ogre::Quaternion& planeOrientation, Ogre::Vector3&
      intersection3D, Ogre::Vector2& intersection2D, double& rayLength);
    
  protected slots:
    void parentInitialized();
    void parentDescriptionChanged(const QString& description);
    void parentVisualAidsShown(bool shown);
    void parentTransparencyEnabled(bool enabled);
  };
};

#endif
