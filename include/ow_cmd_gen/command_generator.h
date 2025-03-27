/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_COMMAND_GENERATOR_H_
#define OPEN_WALKER_COMMAND_GENERATOR_H_

#include <ow_core/interfaces/i_command_generator.h>
#include <ow_core/math.h>

/*!
 * \brief Open Walker command generator module namespace. These classes
 * implement the fusion of the different controllers of the balance and foot
 * compliance modules.
 */
namespace ow_cmd_gen
{

/*!
 * \brief The CommandGenerator class
 *
 * This class implements the CommandGenerator module of the
 * openwalker framework.
 * 
 * It combines the the com, left and right offset with the reference
 * trajectory to compute the commanded cartesian position of the robot.
 */
class CommandGenerator : 
  public ow::ICommandGenerator
{
public:
  typedef ow::ICommandGenerator Base;

protected:
  ow::CartesianState Xcmd_l_w_;       //!< the commanded state of the left foot.
  ow::CartesianState Xcmd_r_w_;       //!< the commanded state of the right foot.
  ow::CartesianState Xcmd_com_w_;     //!< the commanded state of the COM.

public:
  /*!
  * \brief CommandGenerator Default constructor.
  * 
  */
  CommandGenerator();

  // destructor
  virtual ~CommandGenerator();

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState contains the commanded pose of the 
  *    left foot ankle coordinate frame L with respect to 
  *    the world coordinate frame W.
  */
  virtual const ow::CartesianState& Xcmd_l_w() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState contains the commanded pose of the 
  *    right foot ankle coordinate frame R with respect to 
  *    the world coordinate frame W.
  */
  virtual const ow::CartesianState& Xcmd_r_w() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState contains the commanded pose of the 
  *    center of mass coordinate frame COM with respect to 
  *    the world coordinate frame W.
  */
  virtual const ow::CartesianState& Xcmd_com_w() const;

  /*!
   * \brief Update function of the module.
   *
   * \param Xr_l_w
   *    Reference CartesianState of the left foot wrt world frame.
   *
   * \param Xoff_l
   *    Left foot offset.
   *
   * \param Xr_r_w
   *    Reference CartesianState of the right foot wrt world frame.
   *
   * \param Xoff_r
   *    Right foot offset.
   *
   * \param Xr_com_w
   *    Reference CartesianState of the CoM wrt world frame.
   *
   * \param Xoff_com
   *    CoM Offset.
   */
  void update(
    const ow::CartesianState& Xr_l_w,
    const ow::CartesianState& Xoff_l,
    const ow::CartesianState& Xr_r_w,
    const ow::CartesianState& Xoff_r,
    const ow::CartesianState& Xr_com_w,
    const ow::CartesianState& Xoff_com);

protected:
  /*!
   * \brief Initialization of CommandGenerator module
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);
};

}

#endif
