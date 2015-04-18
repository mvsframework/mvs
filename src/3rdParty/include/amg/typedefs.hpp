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


#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <Eigen/Dense>

namespace AMG {
  
  /** \brief A tupel of 3 values, representing a set of coordinates.
   * 
   * The purpose of this type is to allow faster computations by accessing the
   * underlying Eigen::Vector3d type inside functions and algorithms that have
   * sorted out the FrameOfReference issues, i.e. all utilized entities are
   * expressed in the same FrameOfReference.
   */
  using CoordinateTupel = Eigen::Vector3d;
  
  /** \brief A tupel of 4 values, representing a set of quaternions.
   * 
   * Quaternions are normally defined as either <b>q = w + ix + jy + kz</b> or 
   * <b>q = q0 + iq1 + jq2 + kq3</b>. 
   * 
   * Eigen, the underlying matrix library has an own type for quaternions:
   * Eigen::Quaternion::Coefficients. However, those coefficients are
   * internally stored in an unconeventional manner, which could lead to 
   * confusion. Eigen stores the quaternion tupel as [x,y,z,w], which could lead 
   * to an ordering permutation when used in the "flight mechanics way" as
   * 
   * \code
   * double q0=Eigen::Quaternion::Coefficient[3]; // w, the real part
   * double q1=Eigen::Quaternion::Coefficient[0]; // x, the 1st imaginary part
   * double q2=Eigen::Quaternion::Coefficient[1]; // y, the 2nd imaginary part
   * double q3=Eigen::Quaternion::Coefficient[2]; // z, the 3rd imaginary part
   * \endcode
   * 
   * In order to avoid this, an  AMG::QuaternionTupel is defined as the 
   * [q0,q1,q2,q3] tupel of quaternions: 
   * 
   * \code
   * double q0=AMG::QuaternionTupel[0]; // w, the real part
   * double q1=AMG::QuaternionTupel[1]; // x, the 1st imaginary part
   * double q2=AMG::QuaternionTupel[2]; // y, the 2nd imaginary part
   * double q3=AMG::QuaternionTupel[3]; // z, the 3rd imaginary part
   * \endcode
   * 
   * \warning As a result of this 
   *  AMG::QuaternionTupel != Eigen::Quaternion::Coefficients ! (It is a 
   *  permutation of it.)
   */
  using QuaternionTupel = Eigen::Vector4d ;
  
  /** \brief A 3-tupel holding 3-2-1 Euler angles.
   * 
   * This is a 3-tupel holding the roll/phi, pitch/theta, and yaw/psi 
   * Euler-angles.
   * 
   * \note The tupel does not keep track of the used units, i.e. degrees or 
   *  radians! (That's a todo...)
   * 
   * \code
   * AMG::EulerAngleTupel euler;
   * 
   * double phi   = euler[0]; // roll angle
   * double theta = euler[1]; // pitch angle
   * double psi   = euler[2]; // yaw angle
   * \endcode
   * 
   * \todo Make this a template taking Units::degree or Units::radian
   */
  using EulerAngleTupel = Eigen::Vector3d;
  
  
  
}


#endif // TYPEDEFS_H