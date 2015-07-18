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

#ifndef TF_MARKER_CROSSHAIR_H
#define TF_MARKER_CROSSHAIR_H

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <OgreSceneManager.h>
#endif

#include <QColor>
#include <QObject>
#include <QString>

namespace Ogre {
  class Entity;
  class SceneNode;
  class Viewport;
};

namespace rviz {  
  class DisplayContext;
};

namespace rviz_tf_marker {
  class TFMarker;
  
  class TFMarkerCrosshair :
    public QObject,
    public Ogre::SceneManager::Listener,
    public boost::enable_shared_from_this<TFMarkerCrosshair> {
  Q_OBJECT
  public:
    TFMarkerCrosshair(rviz::DisplayContext* context,
      Ogre::SceneNode* parentNode, TFMarker* parent, const QString&
      resource = "package://rviz_tf_marker/resource/crosshair.stl",
      double scale = 1.0, const QColor& color = Qt::white);
    virtual ~TFMarkerCrosshair();

    Ogre::SceneNode* getSceneNode();
    void setResource(const QString& resource);
    void setScale(double scale);
    void setColor(const QColor& color, bool transparent = true);
  
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    TFMarker* parent;
    
    Ogre::Entity* entity;
    Ogre::MaterialPtr material;
    
    QString resource;
    QColor color;
    double scale;
    
    void preFindVisibleObjects(Ogre::SceneManager* source,
      Ogre::SceneManager::IlluminationRenderStage irs,
      Ogre::Viewport* v);
    
  protected slots:
    void parentTransparencyEnabled(bool enabled);
  };
};

#endif
