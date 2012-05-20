#ifndef _WORB_COLLISION_H_INCLUDED
#define _WORB_COLLISION_H_INCLUDED

/**
 *  @file      Collision.h
 *  @brief     Definitions for the Collision class which implements the collision
 *             response for a single collision.
 *  @author    Mikica Kocic
 *  @version   0.18
 *  @date      2012-05-02
 *  @copyright GNU Public License.
 */

#include "RigidBody.h"

namespace WoRB {

    /** Encalpsulates a collision event between two bodies; contains contact details.
     */
    class Collision
    {
        friend class CollisionResolver;

    public:
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Collision state variables                                            */
                                                                                   /*@{*/
        /** Holds the first rigid body that is involved in the collision. 
         */
        RigidBody* Body_A;

        /** Holds the second rigid body that is involved in the collision.
         * It is null in case of the collision with a scenery.
         */
        RigidBody* Body_B;

        /** Holds the position of the contact in world frame of reference.
         */
        Quaternion Position;

        /** Holds the direction of the contact in world frame of reference.
         */
        Quaternion Normal;

        /** Holds the penetration depth at the point of contact.
         */
        double Penetration;

        /** Holds the position projections coefficient for this collision.
         */
        double Restitution;

        /** Holds the friction coefficient for this collision.
         */
        double Friction;

        /** Returns true if the collision is with scenery.
         */
        bool WithScenery () const
        {
            return ! Body_B;
        }

    private:
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Collision response methods                                           */
                                                                                   /*@{*/
        /** Updates the derived quantities from the state data.
         * @note Should be called before the collision response algorithm executes.
         */
        void UpdateDerivedQuantities( double h /*!< Time-step */ )
        {
            if ( ! Body_A ) // Reverse the contact and swap the bodies
            {
                Normal = -Normal; // Reverse the contact normal

                RigidBody* temp = Body_A; // Swap bodies
                Body_A = Body_B; 
                Body_B = temp;
            }

            // Calculate an set of axis at the point of contact.
            //
            FindOrthonormalBasisAtContactPoint ();

            // Find the relative position and velocity relative to each body
            //
            RelativePosition[0] = Position - Body_A->Position;
            Velocity = GetRelativeVelocity( Body_A, RelativePosition[0], h );

            if ( Body_B ) 
            {
                RelativePosition[1] = Position - Body_B->Position;
                Velocity -= GetRelativeVelocity( Body_B, RelativePosition[1], h );
            }

            // Calculate the desired change in velocity for the collision resolution
            //
            BouncingVelocity = GetBouncingVelocity( h );
        }

        /** Applies the linear & angular impulses needed to resolve the collision.
         */
        void ImpulseTransfer( 
            Quaternion V_jolt[2], //!< Returned applied velocity jolt
            Quaternion W_jolt[2]  //!< Returned applied angular velocity jolt
        );

        /** Applies the linear position jolt needed to resolve the collision.
         */
        void PositionProjection( 
            Quaternion X_jolt[2], //!< Returned applied position jolt
            Quaternion Q_jolt[2], //!< Returned applied orientation jolt
            double relaxation     //!< Position projections relaxation coefficient
        );

    private:
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Derived quantities                                                   */
                                                                                   /*@{*/
        /** Contains a transformation matrix from body to world frame of reference.
         */
        QTensor ToWorld;

        /** Holds the relative velocity v_A - v_B between bodies at the point of contact.
         */
        Quaternion Velocity;

        /** Holds the required change in velocity for this contact to be resolved.
         * The basic expression is `-( 1 + COR ) * Velocity.x`
         */
        double BouncingVelocity;

        /** Holds the position of the contact point in world frame, relative to the
         * center of each body.
         */
        Quaternion RelativePosition[2];
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Methods that calculates derived quantities                           */
                                                                                   /*@{*/
        /** Calculates the impulse needed to resolve the collision without friction.
         */
        Quaternion GetImpulse ();

        /** Calculates the impulse needed to resolve the collision in general case.
         */
        Quaternion GetImpulse_IncludeFriction ();

        /** Activates (only) inactive bodies in a collision. 
         * Collisions with the scenery (in case where body B is null) never 
         * cause a body to be activated.
         */
        void ActivateInactiveBodies ()
        {
            if ( Body_B && ( Body_A->IsActive ^ Body_B->IsActive ) )
            {
                if ( Body_A->IsActive ) {
                    Body_B->Activate ();
                }
                else {
                    Body_A->Activate ();
                }
            }
        }

        /** Calculates an orthonormal basis for the contact point.
         */
        void FindOrthonormalBasisAtContactPoint ()
        {
            Quaternion tangent_Y, tangent_Z;

            if ( fabs( Normal.x ) > fabs( Normal.y ) ) // Z-axis is nearer to the Y axis
            {
                double length = 1.0 / sqrt( Normal.z * Normal.z + Normal.x * Normal.x );

                // The new X-axis is at right angles to the world Y-axis
                tangent_Y.x =  Normal.z * length;
                tangent_Y.y =  0;
                tangent_Y.z = -Normal.x * length;

                // The new Y-axis is at right angles to the new X- and Z- axes
                tangent_Z.x =  Normal.y * tangent_Y.x;
                tangent_Z.y =  Normal.z * tangent_Y.x - Normal.x * tangent_Y.z;
                tangent_Z.z = -Normal.y * tangent_Y.x;
                tangent_Z.Normalize ();
            }
            else // Z-axis is nearer to the X axis
            {
                double length = 1.0 / sqrt( Normal.z * Normal.z + Normal.y * Normal.y );

                // The new X-axis is at right angles to the world X-axis
                tangent_Y.x =  0;
                tangent_Y.y = -Normal.z * length;
                tangent_Y.z =  Normal.y * length;

                // The new Y-axis is at right angles to the new X- and Z- axes
                tangent_Z.x =  Normal.y * tangent_Y.z - Normal.z * tangent_Y.y;
                tangent_Z.y = -Normal.x * tangent_Y.z;
                tangent_Z.z =  Normal.x * tangent_Y.y;
                tangent_Z.Normalize ();
            }

            /* Orthonormal basis is a 3x3 matrix, where each vector is a column. 
             * The X-direction is generated from the contact normal, and the Y and Z
             * directions are set so they are at right angles to it.
             */
            ToWorld.SetColumnVectors( Normal, tangent_Y, tangent_Z );
        }

        /** Gets the relative velocity for the point of contact on the given body.
         */
        Quaternion GetRelativeVelocity(
            RigidBody* body,                    //!< The pointer to the rigid body
            const Quaternion& relativePosition, //!< Relative position to the contact
            double h                            //!< The last time-step
            )
        {
            // Calculate the velocity of the contact point in contact coordinates
            //
            Quaternion V_world = body->Velocity 
                               + body->AngularVelocity.Cross( relativePosition );

            Quaternion V = ToWorld.TransformInverse( V_world );

            // Calculate the ammount of velocity that is due to forces without reactions,
            // ignoring any component of acceleration in the contact normal direction 
            // (where only tangential/planar components are considered)
            //
            Quaternion dV_world = body->InverseMass * body->Force * h;

            Quaternion dV = ToWorld.TransformInverse( dV_world );

            dV.x = 0; // consider only tangential components

            // Add the tangential velocities (they will removed during the impulse
            // transfer, if there's enough friction).
            //
            return V + dV;
        }

        /** Calculates the bouncing velocity required to resolve the collision.
         * The bouncing velocity is `-( 1 + COR ) * NormalVelocity.x`.
         */
        double GetBouncingVelocity( double h /*!< Time-step */ )
        {
            // Calculate the normal component of the velocity induced by the force 
            // accumulated in the last time step.
            //
            double dV_fromForce_x = 0;
            if ( Body_A->IsActive )
            {
                Quaternion last_dV = Body_A->InverseMass * Body_A->Force * h;
                dV_fromForce_x += last_dV.Dot( Normal );
            }
            if ( Body_B && Body_B->IsActive )
            {
                Quaternion last_dV = Body_B->InverseMass * Body_B->Force * h;
                dV_fromForce_x -= last_dV.Dot( Normal );
            }

            // Limit the restitution in case when the velocity is very low.
            //
            double COR = fabs( Velocity.x - dV_fromForce_x ) < 0.25 ? 0.0 : Restitution;

            // The result is the bouncing-velocity, reduced for the velocity that was
            // induced by the force exerted in the last time-step.
            //
            return - ( 1 + COR ) * Velocity.x + COR * dV_fromForce_x;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Miscellaneous methods                                                */
                                                                                   /*@{*/
        /** Displays the collision state variables on standard output.
         */
        private: void Dump( unsigned id, double currentTime ) const;
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
    };
} // namespace WoRB

#endif // _WORB_COLLISION_H_INCLUDED
