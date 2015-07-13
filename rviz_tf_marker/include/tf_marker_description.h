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
#endif

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
  class TFMarkerDescription :
    public QObject,
    public boost::enable_shared_from_this<TFMarkerDescription> {
  Q_OBJECT
  public:
    TFMarkerDescription(rviz::DisplayContext* context, Ogre::SceneNode*
      parentNode, double scale = 1.0);
    virtual ~TFMarkerDescription();

    Ogre::SceneNode* getSceneNode();
    void setDescription(const QString& description);
    void setScale(double scale);
  
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    rviz::MovableText* text;
    
    double scale;
  };
};

#endif
