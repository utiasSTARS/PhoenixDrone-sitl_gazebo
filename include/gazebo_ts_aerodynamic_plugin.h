/*
 * gazebo_ts_aerodynamic_plugin.h
 *
 *  Created on: Nov 11, 2017
 *      Author: yilun
 */

#ifndef INCLUDE_GAZEBO_TS_AERODYNAMIC_PLUGIN_H_
#define INCLUDE_GAZEBO_TS_AERODYNAMIC_PLUGIN_H_


#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE TSAeroPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: TSAeroPlugin();

    /// \brief Destructor.
    public: ~TSAeroPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;


    /// \brief Lift Coefficient Lift = K*w*w*delta
    protected: double k_lift;

    /// \brief Drag Coefficient Drag = K*w*w*delta*delta
    protected: double k_drag;

    /// \brief Pitch Moment Coefficient Pitch = K*w*w*delta
    protected: double k_pitch;

    /// \brief center of pressure in link local coordinates
    protected: math::Vector3 cp;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight.
    protected: math::Vector3 forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    protected: math::Vector3 upward;


    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief Pointer to motor joint to read propeller rads.
    protected: physics::JointPtr motorJoint;

    /// \brief Pointer to a joint that actuates a control surface for
    /// this lifting body
    protected: physics::JointPtr controlJoint;


    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;
  };
}


#endif /* INCLUDE_GAZEBO_TS_AERODYNAMIC_PLUGIN_H_ */
