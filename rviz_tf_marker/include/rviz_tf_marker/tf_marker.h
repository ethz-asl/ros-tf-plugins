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
  class TFMarkerArrows;
  class TFMarkerCrosshair;
  class TFMarkerDescription;
  class TFMarkerDisplay;
  class TFMarkerDisc;
  
  class TFMarker :
    public QObject,
    public boost::enable_shared_from_this<TFMarker> {
  Q_OBJECT
  friend class TFMarkerControl;
  public:
    enum Mode {
      modeMove,
      modeRotate,
      modeMoveRotate
    };
    
    TFMarker(rviz::DisplayContext* context, Ogre::SceneNode* parentNode,
      TFMarkerDisplay* parent);
    virtual ~TFMarker();

    Ogre::SceneNode* getSceneNode();
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
    void setYawMode(Mode mode);
    void setShowPitchControls(bool show);
    void setPitchMode(Mode mode);
    void setShowRollControls(bool show);
    void setRollMode(Mode mode);
    void setShowVisualAids(bool show);
    void setPose(const Ogre::Vector3& position, const Ogre::Quaternion&
      orientation);
    void setPose(const geometry_msgs::Pose& message);

    void enableTransparency(bool enable);
    
    void translate(Ogre::Vector3 translation);
    void rotate(Ogre::Quaternion rotation);
  
  signals:
    void initialized();
    void descriptionChanged(const QString& description);
    void visualAidsShown(bool shown);
    void transparencyEnabled(bool enabled);
    void draggingStarted();
    void poseChanged(const Ogre::Vector3& position, const
      Ogre::Quaternion& orientation);
    void draggingStopped();
  
  protected:
    rviz::DisplayContext* context;
    Ogre::SceneNode* sceneNode;
    Ogre::SceneNode* referenceNode;
    Ogre::SceneNode* controlsNode;
    Ogre::SceneNode* positionControlsNode;
    Ogre::SceneNode* orientationControlsNode;
    TFMarkerDisplay* parent;
    
    rviz::Axes* axes;
    boost::shared_ptr<TFMarkerDescription> description;
    boost::shared_ptr<TFMarkerCrosshair> crosshair;
    
    boost::shared_ptr<TFMarkerArrows> xControl;
    boost::shared_ptr<TFMarkerArrows> yControl;
    boost::shared_ptr<TFMarkerArrows> zControl;
    
    boost::shared_ptr<TFMarkerDisc> yawControl;
    boost::shared_ptr<TFMarkerDisc> pitchControl;
    boost::shared_ptr<TFMarkerDisc> rollControl;
    
    mutable boost::recursive_mutex mutex;
    
    Ogre::SceneNode* getReferenceNode();
    
    void startDragging();
    void stopDragging();
    
  protected slots:
    void parentInitialized();
    void parentNameChanged(const QString& name);
  };
};

#endif
