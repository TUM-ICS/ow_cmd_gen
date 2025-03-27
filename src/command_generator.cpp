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

#include <ow_cmd_gen/command_generator.h>
#include <ow_core/math.h>

namespace ow_cmd_gen
{

  CommandGenerator::CommandGenerator() :
    Base("CommandGenerator"),
    Xcmd_l_w_(ow::CartesianState::Zero()),
    Xcmd_r_w_(ow::CartesianState::Zero()),
    Xcmd_com_w_(ow::CartesianState::Zero())
  {
  }

  CommandGenerator::~CommandGenerator()
  {
  }

  bool CommandGenerator::init(const ow::Parameter& parameter, ros::NodeHandle& nh)
  {
    return true;
  }

  void CommandGenerator::update(
    const ow::CartesianState& Xr_l_w,
    const ow::CartesianState& Xoff_l,
    const ow::CartesianState& Xr_r_w,
    const ow::CartesianState& Xoff_r,
    const ow::CartesianState& Xr_com_w,
    const ow::CartesianState& Xoff_com)
  {
    // left foot commanded wrt world
    Xcmd_l_w_.pos() = Xr_l_w.pos()*Xoff_l.pos();

    // right foot commanded wrt world
    Xcmd_r_w_.pos() = Xr_r_w.pos()*Xoff_r.pos();

    // com foot commanded wrt world
    Xcmd_com_w_.pos() = Xr_com_w.pos()*Xoff_com.pos();
  }

  const ow::CartesianState& CommandGenerator::Xcmd_l_w() const
  {
    return Xcmd_l_w_;
  }

  const ow::CartesianState& CommandGenerator::Xcmd_r_w() const
  {
    return Xcmd_r_w_;
  }

  const ow::CartesianState& CommandGenerator::Xcmd_com_w() const
  {
    return Xcmd_com_w_;
  }

}
