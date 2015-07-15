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

#ifndef TF_MARKER_ARROW_H
#define TF_MARKER_ARROW_H

#ifndef Q_MOC_RUN
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#endif

#include <rviz_tf_marker/tf_marker_control.h>

namespace Ogre {
  class SceneNode;
};

namespace rviz {
  class Arrow;
  class DisplayContext;
};

namespace rviz_tf_marker {
  class TFMarkerArrow :
    public TFMarkerControl {
  Q_OBJECT
  public:
    TFMarkerArrow(rviz::DisplayContext* context, Ogre::SceneNode*
      parentNode, TFMarker* parent, const Ogre::Quaternion&
      orientation = Ogre::Quaternion::IDENTITY, const Ogre::Vector3&
      scale = Ogre::Vector3(0.4, 0.3, 0.3), double offset = 0.5,
      const QString& hint = "<b>Left-Click:</b> Move.");
    virtual ~TFMarkerArrow();
    
    void setOrientation(const Ogre::Quaternion& orientation);
    void setScale(const Ogre::Vector3& scale);
    void setOffset(double offset);
    
  protected:
    rviz::Arrow* arrow;
  };
};

#endif
