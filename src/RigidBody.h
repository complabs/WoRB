#ifndef _WORB_RIGID_BODY_H_INCLUDED
#define _WORB_RIGID_BODY_H_INCLUDED

/**
 *  @file      RigidBody.h
 *  @brief     Definitions for the RigidBody class.
 *  @author    Mikica B Kocic
 *  @version   0.4
 *  @date      2012-05-01
 *  @copyright GNU Public License.
 */

#include "Quaternion.h"
#include "QTensor.h"

namespace WoRB {

    /** Encapsulates a rigid body. 
     *
     * Rigid body is the basic simulation object in the World of Bodies (WoRB).
     *
     * Properties:
     *  - inverse mass, and
     *  - inverse moment of inertia
     *
     * State variables: 
     *  - position, 
     *  - orientation, 
     *  - linear momentum, and 
     *  - angular momentum
     *
     * Derived quantities:
     *  - linear and angular velocity, kinetic energy, etc.
     *
     */
    class RigidBody
    {
    public:
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Properties and state variables of the rigid body                     */
                                                                                   /*@{*/
        /** Holds the inverse of the mass of the rigid body. 
         */
        double InverseMass;

        /** Holds the inverse of the body's moment of inertia tensor in body-fixed frame.
         */
        QTensor InverseInertiaBody;

        /** Holds the linear position of the rigid body in world space.
         */
        Quaternion Position;

        /** Holds the angular orientation of the rigid body in world space.
         */
        Quaternion Orientation;

        /** Holds the linear momentum of the rigid body in world space.
         */
        Quaternion LinearMomentum;

        /** Holds the angular momentum of the rigid body in world space.
         */
        Quaternion AngularMomentum;
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Derived quantities                                                   */

        /** Holds a combined translation/rotation matrix for converting from body-fixed 
         * local coordinates into world coordinates.
         *
         * Columns of ToWorld transofmr matrix are base unit vectors (axes) of
         * the rigid body as seen in the world frame of reference.
         */
        QTensor ToWorld;

        /** Holds the inverse inertia tensor of the body in world space.
         */
        QTensor InverseInertiaWorld;

        /** Holds the linear velocity of the rigid body in world space.
         */
        Quaternion Velocity;

        /** Holds the angular velocity of the rigid body in world space.
         */
        Quaternion AngularVelocity;
        
        /** Holds the total angular momentum of the rigid body in world space.
         */
        Quaternion TotalAngularMomentum;

        /** Holds the total kinetic energy of the rigid body.
         */
        double KineticEnergy;

        /** Holds the total potential energy of the rigid body.
         */
        double PotentialEnergy;
                                                                                   /*@{*/
        /** Holds the amount of weighted mean kinetic energy of the body. 
         */
        double AverageKineticEnergy;

        /** Holds the kinetic energy level under which a body will considered stationary.
         */
        double KineticEnergyThreshold;

        /** Indicates whether to impose kinetic energy damping.
         */
        bool KineticEnergyDamping;
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Total force and torque accumulators
         *
         * These variables store the current total force, torque and acceleration of the 
         * rigid body; these quantities are accummulated inbetween time-steps.
         */                                                                        /*@{*/

        /** Holds the accumulated force to be applied at the next integration step.
         */
        Quaternion Force;

        /** Holds the accumulated torque to be applied at the next integration step.
         */
        Quaternion Torque;
                                                                                   /*@}*/
    public:
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Constructors and destructors                                         */
                                                                                   /*@{*/
        RigidBody ()
            : InverseMass( 0 )
            , ToWorld( QTensor::Identity )
            , KineticEnergy( 0 )
            , PotentialEnergy( 0 )
            , AverageKineticEnergy( 0 )
            , KineticEnergyThreshold( 0 )
            , KineticEnergyDamping( false )
            , IsActive( false )
            , CanBeDeactivated( false )
        {
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Integration and simulation methods
         *
         * These methods are used to simulate the rigid body's motion over time.
         */                                                                        /*@{*/

        /** Integrates the rigid body forward in time for the given time-step length.
         */
        void SolveODE( double h )
        {
            if ( ! IsActive ) {
                return;
            }

            // Solve the linear momentum
            //
            LinearMomentum += Force * h;

            // Solve the angular momentum in world space (included gyroscopic effect)
            //
            AngularMomentum += Torque * h;

            // If enabled, remove kinetic energy added through the numerical 
            // instability in the Semi-implicit Euler integrator.
            //
            if ( KineticEnergyDamping ) {
                DampMomentum( h );
            }

            // Derive the linear and the angular velocity
            //
            Velocity = InverseMass * LinearMomentum;
            AngularVelocity = InverseInertiaWorld * AngularMomentum;

            // Calculate the orientation time derivative in world space
            //
            Quaternion OrientationDot = 0.5 * AngularVelocity * Orientation;

            // Solve the linear position
            //
            Position += Velocity * h;

            // Solve the orientation (angular position)
            //
            Orientation += OrientationDot * h;

            // Normalize orientation to versor and calculate derived quantities
            //
            CalculateDerivedQuantities ();

            // Deactivate body, if allowed, when it becomes stationary.
            //
            if ( CanBeDeactivated )
            {
                // Calculate exponential average of the kinetic energy
                //
                double alpha = pow( 0.5, h ); // alpha = 1 / 2^h
                AverageKineticEnergy = alpha * AverageKineticEnergy 
                                     + ( 1 - alpha ) * KineticEnergy;

                if ( AverageKineticEnergy < KineticEnergyThreshold ) {
                    Deactivate ();
                }
                else if ( AverageKineticEnergy > 10 * KineticEnergyThreshold ) {
                    AverageKineticEnergy = 10 * KineticEnergyThreshold;
                }
            }
        }

        /** Normalizes orientation and calculates derived quantities 
         * from the state variables.
         *
         * @param fromMomenta  If true, derive quantites from linear and angular momenta; 
         *                     otherwise, derive quantities from velocities.
         *
         */
        void CalculateDerivedQuantities( bool fromMomenta = true )
        {
            // Normalize the orientation to versor
            //
            Orientation.Normalize ();

            // Setup the transform matrix for the body from orientation and position.
            //
            ToWorld.SetFromOrientationAndPosition( Orientation, Position );

            // Setup the moment of inertia tensor in world frame of reference.
            //
            InverseInertiaWorld = ToWorld( InverseInertiaBody );

            if ( fromMomenta )
            {
                // Calculate the linear and angular velocity from respecitve momenta
                //
                Velocity = InverseMass * LinearMomentum;
                AngularVelocity = InverseInertiaWorld * AngularMomentum;
            }
            else
            {
                // Calculate the linear and angular momentum from repsective velocities
                //
                LinearMomentum  = Mass()  * Velocity;
                AngularMomentum = InverseInertiaWorld.Inverse () * AngularVelocity;
            }

            // Calculate the total angular momentum
            //
            TotalAngularMomentum = Position.Cross( LinearMomentum ) + AngularMomentum;

            // Derive the total kinetic energy
            //
            KineticEnergy = 0.5 * Velocity        .Dot( LinearMomentum  )
                          + 0.5 * AngularVelocity .Dot( AngularMomentum );
        }

        /** Damp linear and angular momentum.
         */
        void DampMomentum( double timeStep )
        {
            const double linearDamping  = 0;
            const double angularDamping = 0.998;

            if ( linearDamping > 0 ) {
                LinearMomentum *= pow( linearDamping, timeStep );
            }
            if ( angularDamping > 0 ) {
                AngularMomentum *= pow( angularDamping, timeStep );
            }
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Property getters and setters                                         */
                                                                                   /*@{*/
        /** Sets the mass of the rigid body.
         */
        void SetupMass( double mass )
        {
            InverseMass = mass == 0 ? 1e+30 
                        : mass >= 1e+30 ? 0.0 : 1.0 / mass;

            KineticEnergyThreshold = 0.3 * mass;
        }

        /** Gets the mass of the rigid body.
         */
        double Mass () const
        {
            return InverseMass == 0 ? 1e+30 
                 : InverseMass >= 1e+30 ? 0.0 : 1.0 / InverseMass;
        }

        /** Returns true if the mass of the body is not-infinite.
         */
        bool IsFiniteMass () const
        {
            return InverseMass > 0.0;
        }

        /** Initializs the position, orientation, velocity and angular velocity
         * of the rigid body and updates other derived quantities.
         */
        void Set_XQVW
        (
            const Quaternion& X,  //!< The initial position
            const Quaternion& Q,  //!< The initial orientation
            const Quaternion& V,  //!< The initial velocity
            const Quaternion& W   //!< The initial angular velocity in world space
            )
        {
            Position        = X;
            Orientation     = Q;
            Velocity        = V;
            AngularVelocity = W;

            CalculateDerivedQuantities( /*fromMomenta*/ false );
        }

        /** Sets the intertia tensor for the rigid body.
         */
        void SetMomentOfInertia( const QTensor& I_body )
        {
            InverseInertiaBody.SetInverseOf( I_body );
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Flags controlling ODE solver activity for the body                   */
                                                                                   /*@{*/
        /** A body can be inactivated to avoid it being updated SolveODE 
         * or affected  by collisions with the scenery.
         */
        bool IsActive;

        /** Controls whether body is allowed to be inactivated.
         */
        bool CanBeDeactivated;

        /** Allows body to move, i.e. enables its ODE to be solved.
         */
        void Activate ()
        {
            if ( ! IsActive ) 
            {
                IsActive = true;
                // Body must have some kinetic energy to avoid immediate deactivation
                AverageKineticEnergy = 2 * 0.3 * Mass(); 
            }
        }

        /** Disallows body to move i.e. disables its ODE to be solved.
         * For deativated body, its velocities are also set to 0.
         */
        void Deactivate ()
        {
            IsActive             = false;
            LinearMomentum       = 0.0;
            AngularMomentum      = 0.0;
            TotalAngularMomentum = 0.0;
            Velocity             = 0;
            AngularVelocity      = 0;
            KineticEnergy        = 0;
            Force                = 0.0;
            Torque               = 0.0;
        }

        /** Sets whether the body is ever allowed to be deactivated.
         */
        void SetCanBeDeactivated( bool flag = true )
        {
            CanBeDeactivated = flag;

            if ( ! CanBeDeactivated && ! IsActive ) {
                Activate ();
            }
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Setters for the force, torque and acceleration
         *
         * These functions set up forces and torques to apply to the rigid body.
         */                                                                        /*@{*/

        /** Clears the forces and torques in the accumulators. 
         */
        void ClearAccumulators ()
        {
            Force  = 0;
            Torque = 0;
            PotentialEnergy = 0;
        }

        /** Adds the given external force to center of mass of the rigid body.
         * An external force acts on the whole body equally and does not activate 
         * inactive bodies.
         */
        void AddExternalForce( const Quaternion& force, double potentialEnergy = 0 )
        {
            Force += force;
            PotentialEnergy += potentialEnergy;
        }

        /** Adds the given internal force to center of mass of the rigid body.
         */
        void AddForce( const Quaternion& force, double potentialEnergy = 0 )
        {
            Force += force;
            PotentialEnergy += potentialEnergy;
            IsActive = true;
        }

        /** Adds the given internal force to the given point on the rigid body.
         */
        void AddForceAtPoint( const Quaternion& worldPoint, 
            const Quaternion& force, double potentialEnergy = 0 )
        {
            Force += force;
            Torque += ( worldPoint - Position ).Cross( force );
            PotentialEnergy += potentialEnergy;
            IsActive = true;
        }

        /** Adds the given internal force to the given point on the rigid body.
         */
        void AddForceAtBodyPoint( const Quaternion& bodyPoint,
            const Quaternion& force, double potentialEnergy = 0  )
        {
            AddForceAtPoint( ToWorld( bodyPoint ), force, potentialEnergy );
        }

        /** Adds the given internal torque to the rigid body.
         */
        void AddTorque( const Quaternion& torque )
        {
            Torque += torque;
            IsActive = true;
        }

        /////////////////////////////////////////////////////////////////////////////////
    };

} // WoRB

#endif // _WORB_RIGID_BODY_H_INCLUDED
