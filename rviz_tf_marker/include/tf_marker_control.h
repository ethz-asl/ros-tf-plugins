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
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <OgreSceneManager.h>
#endif

#include <QCursor>

#include <rviz/interactive_object.h>
#include <rviz/viewport_mouse_event.h>

namespace Ogre {
  class SceneNode;
};

namespace rviz {
  class DisplayContext;
};

namespace rviz_tf_marker {
  class TFMarker;
    
  class TFMarkerControl :
    public Ogre::SceneManager::Listener,
    public rviz::InteractiveObject,
    public boost::enable_shared_from_this<TFMarkerControl> {
  public:
    TFMarkerControl(rviz::DisplayContext* context, Ogre::SceneNode*
      parentNode, TFMarker* parent);
    virtual ~TFMarkerControl();
    
    Ogre::SceneNode* getSceneNode();
    const QCursor& getCursor() const;
    bool isInteractive();
    
    void enableInteraction(bool enable);
    void handleMouseEvent(rviz::ViewportMouseEvent& event);
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    TFMarker* parent;
    
    QCursor cursor;
    QString status;
    
    bool mouseDown;
    bool mouseDragging;
  };
};

#endif
