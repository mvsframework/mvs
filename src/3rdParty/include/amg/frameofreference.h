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


#ifndef FRAMEOFREFERENCE_H
#define FRAMEOFREFERENCE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "typedefs.hpp"
#include "units.hpp"

namespace AMG {

class Vector; //forward declaration
  
/** \brief A coordinate system (or frame of reference) for analytical mechanics.
 * 
 * \todo Work out the different coordinate systems possible (cartesion, 
 *  spherical, geodetic, etc.).
 * \todo A FrameOfReference can have a velocity. Non-intertial frames can even 
 *   have a rotational velocity.
 * 
 */
class FrameOfReference
{

public:
  //NOTE: to see what this is and why it's necessary, read
  // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  /** \brief Constructor for the inital inertial frame of reference.
   * 
   * This constructs the absolute basic inertial frame of reference and hence 
   * can only be called exactly once. Use 
   * \c FrameOfReference(const AMG::FrameOfReference* other); for all subsequent
   * object construction.
   */
  FrameOfReference();
  /** \brief Standart constructor for a frame of reference (copy constructor). */
  FrameOfReference(const AMG::FrameOfReference* other);
  virtual ~FrameOfReference();
  
  friend class Vector;
    
//   friend Eigen::Affine3d FrameOfReference::transformationToRoot(void);
  
  //
  // Accessors
  //
  bool isInertial(void) const; ///< \brief True if this frame is an intertial frame of reference.
  
  /** \brief Get a pointer to the initial inertial frame of reference.
   * \pre Checks if root is not \c NULL.
   */
  static FrameOfReference const * inertialRootFrame(void);
  
    
  /** \brief Get a pointer to the frame's parent frame */
  const FrameOfReference* parentFrame() const;
  
  /** \brief Get the position vector of this frame's origin wrt. its parent frame
   */
  const Vector positionOfOrigin() const;
  
  
  /** \brief Get the quaternion tupel for the current attitude wrt. the parent frame.
   * 
   * The quaternions are defined as <b>q = q0 + iq1 + jq2 + kq3</b>.
   * 
   * \warning Note that Eigen-internally the coefficients are stored in the 
   * following order: [q1, q2, q3, q0], i.e. the "real" parameter is last!
   * 
   * @return The [q1, q2, q3, q0] tupel of 4 quaternion values (all doubles).
   */
  QuaternionTupel attitude() const;
  
  
  /** \brief Get a tupel of yaw, pitch, and roll angles (wrt. the parent frame).
   * 
   * Get the Euler attitude angles as a tupel. 
   * The Euler angles are defined in the 3-2-1/Z-Y'-Z'' flight mechanics 
   * fashion.
   * \code
   * FrameOfReference frame;
   * EulerAngleTupell eulerAngles = frame.attitude(); // Units::degree is the default
   * 
   * double roll  = eulerAngles[0];
   * double pitch = eulerAngles[1];
   * double yaw   = eulerAngles[2];
   * \endcode
   * 
   * \note Due to the way how Eigen internally represents the rotation, there is
   *  a potential for sign errors. Check the note in frameofreference.cpp for 
   *  implementation details. 
   * 
   * \sa EulerRotationMatrix .
   *
   * @return A tupel representing the 3 Euler attitude angles.
   */
  // template<typename units=Units::degree> // NOTE: C++11 feature...
  template<typename units>
  EulerAngleTupel attitude() const;
  
  //
  // Mutators
  //
  /** \brief Set the origin of this frame to the indicated position.
   * The position is given as the tip of the given vector wrt. to the vector's
   * frame of reference.
   * \param[in] absoluteLocation The new position of the origin of this frame.
   */
  void setPositionOfOrigin(const Vector absoluteLocation);
  
  /** \brief Set the attitude of this frame relative to another frame.
   * \tparam units AMG::Units::degree or AMG::Units::radian
   * \param[in] psi The rotation angle around the z-axis
   * \param[in] theta The rotation angle around the y'-axis 
   * \param[in] phi The rotation angle around the x''-axis
   * \param[in] referenceFrame The frame used as an attitude reference. 
   *   (optional; NULL is interpreted as the parent frame.)
   */
  template<typename units>
  void setAttitude(const double psi, const double theta, const double phi,
                   const FrameOfReference* referenceFrame = NULL);

  /** \brief Set the attitude of this frame relative to another frame.
   * \overload
   * \param[in] relativeAttitude Euler angles expressing the new attitude of this
   *  frame relative to the referenceFrame
   * \param[in] referenceFrame The frame used as an attitude reference. 
   *  (optional; NULL is interpreted as the parent frame.)
   * \tparam units AMG::Units::degree or AMG::Units::radian
   */
  template<typename units>
  void setAttitude(const EulerAngleTupel relativeAttitude, const FrameOfReference* referenceFrame = NULL)
  {
   setAttitude<units>(relativeAttitude[2],relativeAttitude[1],relativeAttitude[1], referenceFrame); 
  }
  
  /** \brief Set the attitude of this frame relative to another frame.
   * \overload
   * \param[in] relativeAttitude Quaternions expressing the new attitude of this
   *  frame relative to the referenceFrame
   * \param[in] referenceFrame The frame used as an attitude reference. 
   *  (optional; NULL is interpreted as the parent frame.)
   */ 
  void setAttitude(const QuaternionTupel relativeAttitude, const FrameOfReference* referenceFrame = NULL);
  
  
  /** \brief Translate the frame by the given values relative to itself.
   * This translation moves the frame by the given values. The coordinate tupel
   * is interpreted relative to this frame's coordinate system.
   * 
   * A translation essentially effects the offset of the origin of this 
   * coordinate system from the origin of the parent's coordinate system.
   * In order to keep track of the overall offset, each reference frame stores 
   * the accumulated translations in \c translationFromParent. To do so, this 
   * function first converts the given translation vector from this (the local)
   * coordinate frame to the parent's coordinates frame. After that, the 
   * translations are added together. 
   * 
   * \pre Ensures the translated frame not to be the root frame.
   * 
   * @param[in] xTranslation Translate by this amount in the direction of the frame's x-axis
   * @param[in] yTranslation Translate by this amount in the direction of the frame's y-axis
   * @param[in] zTranslation Translate by this amount in the direction of the frame's z-axis
   */
  void translate(const double& xTranslation ,
                 const double& yTranslation ,
                 const double& zTranslation );
  
  /** \brief Translate the frame by the given coordinates relative to itself.
   * \overload
   * @param[in] xyzTupel A tupel of three values for x-, y-, and z-axis translation
   */
  void translate(const CoordinateTupel& xyzTupel);
  
  /** \brief Translate the frame by the given vector.
   * \overload
   * \param[in] vector A vector representing the translation.
   */
  void translate(const Vector& vector);
  
  
  /** \brief Get the roll attitude angle (wrt. the parent frame). */
  // template<typename units=Units::degree> // NOTE: C++11 feature...
  template<typename units>
  double roll() const
  {
    CoordinateTupel euler(attitude<units>());
    return euler[0];
  };
  
  /** \brief Get the pitch attitude angle (wrt. the parent frame). */
  // template<typename units=Units::degree> // NOTE: C++11 feature...
  template<typename units>
  double pitch() const
  {
    CoordinateTupel euler(attitude<units>());
    return euler[1];
  };
  
  /** \brief Get the yaw attitude angle (wrt. the parent frame). */
  // template<typename units=Units::degree> // NOTE: C++11 feature...
  template<typename units>
  double yaw() const
  {
    CoordinateTupel euler(attitude<units>());
    return euler[2];
  };

  /** \brief A 3-2-1 Euler-rotation of the frame.
   * This rotation follows the standart Euler-angle definition for flight 
   * mechanics.
   * 
   * \pre Ensures the rotated frame not to be the root frame.
   * \pre -M_PIl <= psi_rad < M_PIl
   * \pre -M_PI_2l <= theta_rad <= M_PI_2l
   * \pre -M_PIl <= phi_rad < M_PIl
   * 
   * \todo Make it take long doubles (and propagate that down the list)
   * 
   * @param[in] psi The rotation angle around the z-axis.
   * @param[in] theta The rotation angle around the y'-axis.
   * @param[in] phi The rotation angle around the x''-axis.
   */
  // template<typename units=Units::degree> // NOTE: C++11 feature...
  template<typename units>
  void rotate (const double& psi   ,
               const double& theta ,
               const double& phi   );
  
  /** \brief Convert a coordinate tupel from this frame to the parent's frame
   */
  CoordinateTupel expressInParentFrame (const AMG::CoordinateTupel& tupel) const;

//TODO: Work out the different coordinate systems, i.e. cartesion, spherical, etc..  
//  
//   /* \overload
//    * 
//    * \todo Look into order of non-cartesian coordinate systems
//    * 
//    * @param[in] c1 The 1st-dimension coordinate 
//    * @param[in] c2 The 2nd-dimension coordinate
//    * @param[in] c3 The 3rd-dimension coordinate    
//    */
//   CoordinateTupel expressInParentFrame (const double& c1,
//                                         const double& c2,
//                                         const double& c3) const;
  
  /** \brief A debug function to print the frames orientation wrt. the parent. */
  void outputRelativePositionToParent(void) const;
  
  /** \brief Construct the flight mechanics standart 3-2-1 Euler rotation matrix.
   * This function returns the standart rotation matrix which translates a 
   * vector expressed in the vehicle carried NED frame to the body frame of an
   * aircraft with the given attitude of roll/phi, pitch/theta, and yaw/psi.
   * 
   * v_b = L_ba*v_a
   * 
   * Where:
   *    v_a  : A vector expressed in the \c a frame of reference, e.g. NED
   *    v_b  : A vector expressed in the \c b frame of reference, e.g body-frame
   *    L_ba : The matrix rotating the frame \c a into the frame \c b.
   * 
   * \pre -M_PIl <= psi < M_pil
   * \pre -M_PI_2l <= theta <= M_PI_2l
   * \pre -M_PIl <= phi < M_PIl
   * 
   * \note Due to the way how Eigen internally represents the rotation, there is
   *  a potential for sign errors. Check the note in frameofreference.cpp for 
   *  implementation details.
   * 
   * \todo Make this a function template taking units as \c AMG::degree or 
   *    \c AMG::radian.
   */
  static Eigen::Matrix3d EulerRotationMatrix(const double& psi,
                                             const double& theta,
                                             const double& phi);
  
  
  /** \brief Convert 4 quaternion parameters to Euler angles.
   * 
   * The quaternions are defined as <b>q = q0 + iq1 + jq2 + kq3</b>.
   * 
   * @param[in] q0 The first quaternion.
   * @param[in] q1 The second quaternion.
   * @param[in] q2 The third quaternion.
   * @param[in] q3 The fourth quaternion.
   * @param[out] psi Yaw angle in units.
   * @param[out] theta Pitch angle in units.
   * @param[out] phi Roll angle in units.
   */
  template<typename units=Units::degree>
  static void quaternions2euler(const double& q0    ,
                                const double& q1    ,
                                const double& q2    ,
                                const double& q3    ,
                                      double& psi   ,  // yaw
                                      double& theta ,  // pitch
                                      double& phi   );  // roll
  
  /** \brief  Convert 4 quaternion parameters to Euler angles.
   * \overload
   */
  template<typename units=Units::degree>
  static void quaternions2euler(const QuaternionTupel quaternions ,
                                EulerAngleTupel       eulerAngles )
  {
    quaternions2euler<units>(quaternions[0],
                             quaternions[1],
                             quaternions[2],
                             quaternions[3],
                             eulerAngles.coeffRef(2),//psi_<units>
                             eulerAngles.coeffRef(1),//theta_<units>
                             eulerAngles.coeffRef(0)//phi`_<units> 
                            );    
  };


  
  
  /** \brief Outputs the offset from the parent frame to the given stream. */
  friend std::ostream& operator<< ( std::ostream&  stream, const FrameOfReference& f ); 

protected:  
  
  /** \brief Is this an inertial frame of reference?
   * \see http://en.wikipedia.org/wiki/Inertial_frame_of_reference
   */
  bool inertial; 
  
  
  /** \brief The frame of reference this one was created from.
   */
  const FrameOfReference* parent; 


private:
  /** \brief Private assingment operator to dissallow assignment. */
  virtual FrameOfReference& operator=(const FrameOfReference* other) {return *this;};

  /** \brief Private equal-to operator to dissalow comparison  */
  virtual bool operator==(const FrameOfReference& other) const {return false;}; 
  
  /** \brief The absolute initial (and hence inertial) frame of reference.
   * This pointer serves a dual purpose:
   * \li Pointer to the beginning ;) 
   * \li Allow the empty constructor to check whether it has been called before
   */
  static const FrameOfReference* root;
  
  /** \brief The translations(s) performed on this reference frame.
   */
  Eigen::Translation3d translationFromParent;
  
  /** \brief The rotation(s) performed on this reference frame.
   */
  Eigen::Matrix3d rotationFromParent;
  
  /** \brief Get the combined transformation from the local frame to the parent.
   */
  Eigen::Affine3d transformationToParent(void) const; 
  
  Eigen::Affine3d transformationToRoot(void) const;
  

  
};





template<>
inline void FrameOfReference::quaternions2euler<Units::radian>(const double& q0,
                                                        const double& q1,
                                                        const double& q2,
                                                        const double& q3,
                                                        double& psi,
                                                        double& theta,
                                                        double& phi)
{
  phi   = std::atan2( 2.0*(q0*q1+q2*q3), 1.0-2.0*(q1*q1+q2*q2) );
  theta = std::asin( 2.0*(q0*q2-q3*q1) );
  psi   = std::atan2( 2.0*(q0*q3+q1*q2), 1.0-2.0*(q2*q2+q3*q3) );
};

template<>
inline void FrameOfReference::quaternions2euler<Units::degree>(const double& q0,
                                                        const double& q1,
                                                        const double& q2,
                                                        const double& q3,
                                                              double& psi,
                                                              double& theta,
                                                              double& phi)
{
 quaternions2euler<Units::radian>(q0,q1,q2,q3,psi,theta,phi);
 
 psi   = Units::radian2degree(psi);
 theta = Units::radian2degree(theta);
 phi   = Units::radian2degree(phi);
};


} // namespace AMG

#endif // FRAMEOFREFERENCE_H
