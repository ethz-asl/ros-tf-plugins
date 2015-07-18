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

#ifndef TF_MARKER_DISC_H
#define TF_MARKER_DISC_H

#ifndef Q_MOC_RUN
#include <vector>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#endif

#include <rviz_tf_marker/tf_marker_control.h>

namespace Ogre {
  class ManualObject;
  class SceneNode;
};

namespace rviz {
  class DisplayContext;
};

namespace rviz_tf_marker {
  class TFMarkerDisc :
    public TFMarkerControl {
  Q_OBJECT
  public:
    TFMarkerDisc(rviz::DisplayContext* context, Ogre::SceneNode*
      parentNode, TFMarker* parent, const QString& name, const
      Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY,
      const Ogre::Vector3& scale = Ogre::Vector3::UNIT_SCALE,
      double radius = 0.66, size_t numSegments = 36, const QString&
      hint = "<b>Left-Click:</b> Move/Rotate.");
    virtual ~TFMarkerDisc();
    
    void setOrientation(const Ogre::Quaternion& orientation);
    void setScale(const Ogre::Vector3& scale);
    void setTransparent(bool transparent);
    
  protected:
    Ogre::ManualObject* manualObject;
    Ogre::MaterialPtr material;
    
    double radius;
    size_t numSegments;
    bool transparent;
    
    void makeCircle(std::vector<Ogre::Vector3>& vertices, double radius,
      size_t numSegments);
    void makeDisc(std::vector<Ogre::Vector3>& vertices,
      std::vector<Ogre::ColourValue>& colors, double innerRadius, double
      outerRadius, size_t numSegments, const QColor& color);
    void updateObject(const Ogre::Quaternion& orientation, bool transparent);
  };
};

#endif
