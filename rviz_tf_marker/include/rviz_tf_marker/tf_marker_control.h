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

#include <rviz/interactive_object.h>
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
      parentNode, TFMarker* parent, const QString& hint = QString());
    virtual ~TFMarkerControl();
    
    Ogre::SceneNode* getSceneNode();
    const QCursor& getCursor() const;
    QString getStatus() const;
    void setHighlight(double ambient);
    bool isInteractive();

    void enableInteraction(bool enable);
    void handleMouseEvent(rviz::ViewportMouseEvent& event);
  
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    TFMarker* parent;
    
    boost::shared_ptr<rviz::SelectionHandler> selectionHandler;
  
    Ogre::Viewport* dragViewport;    
    std::set<Ogre::MaterialPtr> materials;
    std::set<Ogre::Pass*> highlightPasses;
  
    QCursor cursor;
    QString hint;
    
    bool interactionEnabled;
    bool mouseDown;
    bool mouseDragging;
    
    void addMaterial(const Ogre::MaterialPtr& material);
    
    void stopDragging(bool force = false);
    
    void orientationToColor(const Ogre::Quaternion& orientation,
      QColor& color);
    
  protected slots:
    void parentInitialized();
    void parentDescriptionChanged(const QString& description);
  };
};

#endif
