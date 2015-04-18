// 
//  Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// 


#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "frameofreference.h"
#include "vector.h"

#include "commoncoordinatesystems.h"


#include <Eigen/Dense>

namespace AMG {

 /** \brief A physical, rigid body with mass, inertia, and a position in space.
  * A rigid body also, per definition, represents a FrameOfReference.
  * 
  * \todo Implement all kinds of functions, like setMass, setInertialMatrix, etc.
  * \todo A RigidBody is most likely something that will be "simulated", i.e. 
  *   it's position will change rather frequently. As such it might be usefull 
  *   (particularly for debugging), to store the position in explicit variables
  *   instead of an indirect storage as the transformation of the (0,0,0)
  *   body-frame vector... (But maybe this should be implemented already in the
  *   FrameOfReference class???)
  */
class RigidBody : public AMG::FrameOfReference
{

public:
  /** \brief Default (empty) constructor.
   * 
   * This constructs a point mass of 1kg.
   * 
   * 
   * \todo When constructing a body, make sure that the parent frame is indeed 
   *  an inertial frame. If not, get the parent of that frame and continue until
   *  an intertial frame is found (which will eventually be the case when 
   *  FrameOfReference::root has been reached).
   * 
   * @param[in] parentFrame The frame this body is positioned in.
   *  Although a rigid body constitutes a frame of reference and as such should
   *  follow the semantics of one, the constructor takes a pointer (and not a
   *  reference, as the constructor of a FrameOfReference) as this allows the
   *  \c parentFrame to change to a different frame, i.e. in cases where a body
   *  needs to be referenced in a different frame.
   */
  RigidBody(const FrameOfReference* parentFrame=CoSy::getDatumFrame());
  
  /** \brief Constructor with mass and inertia matrix.
   *
   * The inertiaMatrix defaults to Zero(), making the result of this constructor
   * call a point mass with mass \c bodyMass.
   * 
   * @param[in] bodyMass The mass of this body (in kilogramm [kg])
   * @param[in] bodyInertiaMatrix (optional) The inertia matrix of this body (in
   *  [kg*m^2])
   * @param[in] parentFrame The frame this body is positioned in.
   *  Although a rigid body constitutes a frame of reference and as such should
   *  follow the semantics of one, the constructor takes a pointer (and not a
   *  reference, as the constructor of a FrameOfReference) as this allows the
   *  \c parentFrame to change to a different frame, i.e. in cases where a body
   *  needs to be referenced in a different frame.
   */
  RigidBody(const double& bodyMass                                            ,
            const Eigen::Matrix3d & bodyInertiaMatrix=Eigen::Matrix3d::Zero() ,
            const FrameOfReference* parentFrame=CoSy::getDatumFrame()         );
  

  
  /** \brief Destructor */
  virtual ~RigidBody();
  

  
  //
  // Accessors
  //  
  
  Vector position() const;
  Vector velocity() const;
  Vector angularRates() const;
    
  
  //
  // Mutators
  // 
  
  /** \brief Move the body to the given absolute target location in space.
   * 
   * This is not a simulated motion, this is an instantaneous "teleportation".
   * 
   * @param[in] absoluteTargetLocation A Vector pointing to the absolute target 
   *  position. The targetLocation Vector is internally translated using the 
   *  Vector::absoluteCoordsIn function, i.e. origin offsets in between defining
   *  frames are incoorporated. 
   */
  virtual void setPositionTo(const Vector& absoluteTargetLocation);
  
  /** \overload
   * This function overlaod takes geodetic coordinates as arguments and moves 
   * the body to that location.
   * 
   * @param[in] longitude The geodetic longitude of the target position.
   * @param[in] latitude The geodetic latitude of the target position.
   * @param[in] elevation The elevation in meters above the WGS84 ellipsoid.
   */
  template<typename units=AMG::Units::degree>
  void setPositionTo(const double& longitude,
              const double& latitude,
              const double& elevation);
  
  /** \brief Move the body relative to itself.
   * 
   * This is not a simulated motion, this is an instantaneous "teleportation".
   * 
   * @param[in] relativeMotion A Vector indicating the desired motion relative 
   *  to itself. The relativeMotion Vector is internally translated using the 
   *  Vector::coordsIn function, i.e. the origin offsets in between defining
   *  frames are ignored.
   */
  void movePositionBy(const Vector& relativeMotion);
  
   
  void setVelocityTo(const Vector& velocity);
    
  void setRatesTo(const Vector& rotationRates);
  
protected:  
  /** \brief The mass of this body in kilogram [kg]. */
  double mass;
  
  /** \brief The inertia matrix given the body frame. */
  Eigen::Matrix3d inertiaMatrix;
  
  
  // States
  double m_x, m_y, m_z;
  double m_u, m_v, m_w;
  double m_phi, m_theta, m_psi;
  double m_p, m_q, m_r;

  
public:   
  //
  // States
  // 
 
  const double& x; ///< \brief X coordinated of the position of the origin of this body wrt. to the parent frame
  const double& y; ///< \brief Y coordinated of the position of the origin of this body wrt. to the parent frame
  const double& z; ///< \brief Z coordinated of the position of the origin of this body wrt. to the parent frame
  
  const double& u; ///< \brief x-axis velocity component.
  const double& v; ///< \brief y-axis velocity component.
  const double& w; ///< \brief z-axis velocity component.

  const double& phi;    ///< \brief The rotation angle around the x''-axis
  const double& theta;  ///< \brief The rotation angle around the y'-axis
  const double& psi;    ///< \brief The rotation angle around the z-axis
  
  const double& p; ///< \brief Body roll rate
  const double& q; ///< \brief Body pitch rate
  const double& r; ///< \brief Body yaw rate
      
private:
//   /** \brief Unimplemented copy constructor to dissallow copying*/
//   RigidBody(const RigidBody& other){};
  
  /** \brief Private assignement operator to dissallow assignment */
  virtual RigidBody& operator=(const RigidBody& other){return *this;};
  virtual AMG::FrameOfReference& operator=(const AMG::FrameOfReference& other){return *this;};
  
  /** \brief Proivate equal-to operator to dissallow comparison*/
  virtual bool operator==(const AMG::FrameOfReference& other) const {return false;} ;
  virtual bool operator==(const RigidBody& other) const {return false;} ;
  
};







} // end namespace AMG

#endif // RIGIDBODY_H
