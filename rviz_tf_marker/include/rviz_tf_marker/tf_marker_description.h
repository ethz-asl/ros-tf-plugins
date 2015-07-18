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

#ifndef TF_MARKER_DESCRIPTION_H
#define TF_MARKER_DESCRIPTION_H

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#endif

#include <QColor>
#include <QObject>
#include <QString>

namespace Ogre {  
  class SceneNode;
};

namespace rviz {  
  class DisplayContext;
  class MovableText;
};

namespace rviz_tf_marker {
  class TFMarker;
  
  class TFMarkerDescription :
    public QObject,
    public boost::enable_shared_from_this<TFMarkerDescription> {
  Q_OBJECT
  public:
    TFMarkerDescription(rviz::DisplayContext* context, Ogre::SceneNode*
      parentNode, TFMarker* parent, const QString& text = QString(),
      double scale = 1.0, const QColor& color = Qt::white);
    virtual ~TFMarkerDescription();

    Ogre::SceneNode* getSceneNode();
    void setText(const QString& text);
    QString getText() const;
    void setScale(double scale);
    void setColor(const QColor& color);
  
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    TFMarker* parent;
    
    Ogre::SceneNode* textNode;
    rviz::MovableText* text;
    
    QColor color;
    double scale;
  protected slots:
    void parentPoseChanged(const Ogre::Vector3& position, const
      Ogre::Quaternion& orientation);
  };
};

#endif
