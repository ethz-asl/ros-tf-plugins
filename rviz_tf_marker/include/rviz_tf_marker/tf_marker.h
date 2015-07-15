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

#include <geometry_msgs/Pose.h>
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
  class TFMarkerArrow;
  class TFMarkerCrosshair;
  class TFMarkerDescription;
  class TFMarkerDisplay;
  class TFMarkerDisc;
  
  class TFMarker :
    public QObject,
    public boost::enable_shared_from_this<TFMarker> {
  Q_OBJECT
  public:
    TFMarker(rviz::DisplayContext* context, Ogre::SceneNode* parentNode,
      TFMarkerDisplay* parent);
    virtual ~TFMarker();

    void setDescription(const QString& description);
    QString getDescription() const;
    
    void setShowDescription(bool show);
    void setDescriptionColor(const QColor& color);
    void setShowAxes(bool show);
    void setShowCrosshair(bool show);
    void setCrosshairColor(const QColor& color);
    void setShowControls(bool show);
    void setShowPositionControls(bool show);
    void setShowXControls(bool show);
    void setShowYControls(bool show);
    void setShowZControls(bool show);
    void setShowOrientationControls(bool show);
    void setShowYawControls(bool show);
    void setShowPitchControls(bool show);
    void setShowRollControls(bool show);
    void setPose(const Ogre::Vector3& position, const Ogre::Quaternion&
      orientation);
    void setPose(const geometry_msgs::Pose& message);
  
  signals:
    void initialized();
    void descriptionChanged(const QString& description);
  
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    Ogre::SceneNode* controlsNode;
    Ogre::SceneNode* positionControlsNode;
    Ogre::SceneNode* orientationControlsNode;
    TFMarkerDisplay* parent;
    
    rviz::Axes* axes;
    boost::shared_ptr<TFMarkerDescription> description;
    boost::shared_ptr<TFMarkerCrosshair> crosshair;
    
    boost::shared_ptr<TFMarkerArrow> positiveXControl;
    boost::shared_ptr<TFMarkerArrow> negativeXControl;
    boost::shared_ptr<TFMarkerArrow> positiveYControl;
    boost::shared_ptr<TFMarkerArrow> negativeYControl;
    boost::shared_ptr<TFMarkerArrow> positiveZControl;
    boost::shared_ptr<TFMarkerArrow> negativeZControl;
    
    boost::shared_ptr<TFMarkerDisc> yawControl;
    boost::shared_ptr<TFMarkerDisc> pitchControl;
    boost::shared_ptr<TFMarkerDisc> rollControl;
    
    mutable boost::recursive_mutex mutex;
    
  protected slots:
    void parentInitialized();
  };
};

#endif
