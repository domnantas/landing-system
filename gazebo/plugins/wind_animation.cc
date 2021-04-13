/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class WindAnimation : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      double multiplier;

      this->LoadParam(_sdf, "multiplier", multiplier, 1.0);

      // Store the pointer to the model
      this->model = _parent;

      // create the animation
      gazebo::common::PoseAnimationPtr anim(
          // make animation last 5 seconds,
          // and set it on a repeat loop
          new gazebo::common::PoseAnimation("wind", 5.0, true));

      gazebo::common::PoseKeyFrame *key;

      // set starting location of the box
      key = anim->CreateKeyFrame(0);
      key->Rotation(ignition::math::Quaterniond(0.11 * multiplier, -0.07 * multiplier, 0));

      key = anim->CreateKeyFrame(1.0);
      key->Rotation(ignition::math::Quaterniond(0.07 * multiplier, 0.0 * multiplier, 0));

      key = anim->CreateKeyFrame(2.0);
      key->Rotation(ignition::math::Quaterniond(0.11 * multiplier, 0.06 * multiplier, 0));

      key = anim->CreateKeyFrame(3.0);
      key->Rotation(ignition::math::Quaterniond(0.13 * multiplier, 0.13 * multiplier, 0));

      key = anim->CreateKeyFrame(4.0);
      key->Rotation(ignition::math::Quaterniond(0.09 * multiplier, -0.03 * multiplier, 0));

      // set final location equal to starting location
      key = anim->CreateKeyFrame(5.0);
      key->Rotation(ignition::math::Quaterniond(0.11 * multiplier, -0.07 * multiplier, 0));

      // set the animation
      _parent->SetAnimation(anim);
    }

    // Pointer to the model
  private:
    physics::ModelPtr model;

    // Pointer to the update event connection
  private:
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WindAnimation)
}
