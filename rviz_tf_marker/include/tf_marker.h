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

#ifndef TF_MARKER_H
#define TF_MARKER_H

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <boost/thread/recursive_mutex.hpp>

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <geometry_msgs/PoseStamped.h>
#endif

#include <QColor>
#include <QObject>

namespace Ogre {  
  class SceneNode;
};

namespace rviz {
  class Axes;
  class DisplayContext;
  class MarkerBase;
};

namespace rviz_tf_marker {
  class TFMarkerControl;
  class TFMarkerCrosshair;
  class TFMarkerDescription;
  
  class TFMarker :
    public QObject,
    public boost::enable_shared_from_this<TFMarker> {
  Q_OBJECT
  public:
    TFMarker(rviz::DisplayContext* context, Ogre::SceneNode* parentNode);
    virtual ~TFMarker();

    void setDescription(const QString& description);
    
    void setShowDescription(bool show);
    void setShowAxes(bool show);
    void setShowCrosshair(bool show);
    void setCrosshairColor(const QColor& color);
    void setShowVisualAids(bool show);
    void setPose(const Ogre::Vector3& position, const Ogre::Quaternion&
      orientation);
    void setPose(const geometry_msgs::PoseStamped& message);
    
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    
    rviz::Axes* axes;
    boost::shared_ptr<TFMarkerDescription> description;
    boost::shared_ptr<TFMarkerCrosshair> crosshair;
    boost::shared_ptr<TFMarkerControl> control;
    
    boost::recursive_mutex mutex;
  };
};

#endif
