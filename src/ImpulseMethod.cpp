/**
 *  @file      ImpulseMethod.cpp
 *  @brief     Implementation of the impulse transfer method algorithm.
 *  @author    Mikica Kocic
 *  @version   0.18
 *  @date      2012-05-02
 *  @copyright GNU Public License.
 */

#include "WoRB.h"

using namespace WoRB;

/////////////////////////////////////////////////////////////////////////////////////////
// Resolves collisions in the system using the impulse transfer method.
//
void CollisionResolver::ImpulseTransfers( double h, unsigned maxIterations, double eps )
{
    if ( CollisionCount == 0 ) {
        return; // Nothing to do
    }

    // Setup default parameters
    //
    if ( maxIterations == 0 ) {
        maxIterations = 8 * CollisionCount;
    }
    if ( eps == 0 ) {
        eps = 0.01;
    }

    // Iterate performing impulse transfers, until there are no contacts with
    // notable bouncing velocity jolts are found.
    //
    for ( unsigned iteration = 0; iteration < maxIterations; ++iteration )
    {
        // Find the contact with the largest possible velocity jolt
        //
        Collision* contact = FindLargestBouncingVelocity( eps );
        if ( ! contact ) {
            break; // Done, if bouncing velocity are not found
        }

        // Activate bodies participating in the collision that are lying inactive
        //
        contact->ActivateInactiveBodies ();

        // Calculate velocity and angular velocity jolts
        //
        Quaternion V_jolt[2], W_jolt[2];
        contact->ImpulseTransfer( V_jolt, W_jolt );

        // With the change in velocity of the two bodies, the update of
        // contact velocities means that some of the relative closing
        // velocities need recomputing.
        //
        RigidBody** bodies_in_contact = &contact->Body_A;
        for( unsigned i = 0; i < CollisionCount; ++i )
        {
            Collision& c_i = Collisions[i];
            RigidBody** b_i = &c_i.Body_A;

            for( unsigned a = 0; a < 2; ++a ) // Each body in contact
            {
                if ( ! b_i[a] ) { 
                    continue; // Skip scenery objects
                }

                // Check for a match with each body in the newly resolved contact
                //
                for( unsigned b = 0; b < 2; ++b )
                {
                    if ( b_i[a] != bodies_in_contact[b] ) {
                        continue;
                    }

                    // dV = V_j + ( W_j x r )
                    //
                    Quaternion delta_V = 
                        V_jolt[b] + W_jolt[b].Cross( c_i.RelativePosition[a] );

                    // The sign of the change is negative if we're dealing
                    // with the second body in a contact.
                    //
                    Quaternion dV_world = c_i.ToWorld.TransformInverse( delta_V );
                    c_i.Velocity += a ? -dV_world : dV_world;

                    // Recalculate bouncing velocity (derived quantity).
                    // BouncingVelocity = - ( 1 + COR ) * Velocity.x
                    // where Velocity.x = < V_ab, Normal_ab >
                    //
                    c_i.BouncingVelocity = c_i.GetBouncingVelocity( h );
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Impulse transfer for the single collision
//
void Collision::ImpulseTransfer(
        Quaternion V_jolt[2], // calculated linear velocity change
        Quaternion W_jolt[2]  // calculated angular velocity change
    )
{
    // Get the collision impulse in contact frame of reference
    //
    Quaternion J_contact = Friction == 0
        ? GetImpulse ()  // Use the simplifed version in case of frictionless contacts
        : GetImpulse_IncludeFriction ();  // The fuull version including friction

    // Convert impulse to world space, then split it into linear and angular components
    //
    Quaternion J        = ToWorld( J_contact );
    Quaternion J_torque = RelativePosition[0].Cross( J );

    // Apply the jolts on the first body
    //
    Body_A->LinearMomentum  += J;
    Body_A->AngularMomentum += J_torque;

    V_jolt[0] = Body_A->InverseMass * J;
    W_jolt[0] = Body_A->InverseInertiaWorld * J_torque;

    // Apply the jolts on the second body, if it's not a scenery
    //
    if ( Body_B ) 
    {
        J_torque = RelativePosition[1].Cross( J );

        Body_B->LinearMomentum  -= J;
        Body_B->AngularMomentum -= J_torque;

        V_jolt[1] = -( Body_B->InverseMass * J );
        W_jolt[1] = -( Body_B->InverseInertiaWorld * J_torque );
    }

    // Note: The linear and the angular velocity will be derived later
    // in RigidBody::CalculateDerivedQuantities() method.
}

Quaternion Collision::GetImpulse ()
{
    // Build a vector that shows the change in velocity in world space for 
    // a unit impulse in the direction of the contact normal, then calculate the change 
    // in velocity in contact coordiantes, adding also the linear component of 
    // velocity jolt (body inverse mass 1/M).
    //
    // Equation (8-18) in 
    // Baraff, David - Physically Based Modeling, Rigid Body Simulation
    //
    //                 - ( 1 + COR ) * Velocity.x                     Bouncing velocity
    // j =  ------------------------------------------------------ = -------------------
    //       Sum( m^-1 + [ ( I_world^-1 • ( r × N ) ) x r ] • N )     Inv. reduced mass
    //

    // First, calculate:
    // iMass = Sum( m^-1 + ( ( I_world^-1 • ( r × N ) ) x r ) • N ) for each body
    //
    double invRedMass = 0;
    
    invRedMass += Body_A->InverseMass;
    invRedMass += ( Body_A->InverseInertiaWorld 
                    * ( RelativePosition[0].Cross( Normal ) )
                  ).Cross( RelativePosition[0] ).Dot( Normal );

    if ( Body_B ) {
        invRedMass += Body_B->InverseMass;
        invRedMass += ( Body_B->InverseInertiaWorld 
                        * ( RelativePosition[1].Cross( Normal ) )
                      ).Cross( RelativePosition[1] ).Dot( Normal );
    }

    // Finally, return the correct sized impulse in the contact frame of reference
    // along the contact normal (= X-component). 
    //
    return SpatialVector( BouncingVelocity / invRedMass, 0, 0 );
}

Quaternion Collision::GetImpulse_IncludeFriction ()
{
    // Build the matrix for converting between linear and angular quantities.
    // (Multiplying by a skew symmetrix matrix is equivalent to a cross product.)
    //
    QTensor crossR;
    crossR.SetSkewSymmetric( RelativePosition[0] );

    // Build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
    //
    // dV_world = - ( r × I_world^-1 ) × r
    //
    QTensor delta_V_world = -( crossR * Body_A->InverseInertiaWorld * crossR );

    // Do the same for the second body
    //
    if ( Body_B )
    {
        // Calculate the velocity jolt matrix for body B and sum with the velocity
        // jolt for body A
        //
        // dV_world = - ( r_a × I_a_world^-1 ) × r_a - ( r_b × I_b_world^-1 ) × r_b
        //
        crossR.SetSkewSymmetric( RelativePosition[1] ); // cross product matrix
        delta_V_world += -( crossR * Body_B->InverseInertiaWorld * crossR );
    }

    // Do a change of basis to convert into contact coordinates.
    //
    QTensor delta_V_contact = ToWorld.TransformInverse( delta_V_world );

    // Include the linear effects of the inverse reduced mass m_ab^-1
    //
    // m_ab^-1 = m_a^-1 + m_b^-1
    //
    double inverseReducedMass = 
        Body_A->InverseMass + ( Body_B ? Body_B->InverseMass : 0.0 );

    delta_V_contact.m.xx += inverseReducedMass;
    delta_V_contact.m.yy += inverseReducedMass;
    delta_V_contact.m.zz += inverseReducedMass;

    if ( Friction == 0 ) {
        return Quaternion( 0, BouncingVelocity / delta_V_contact.m.xx, 0, 0 );
    }

    // For static friction we should just remove tangential components.
    //
    #if 0
        return Quaternion( 0,
            BouncingVelocity / delta_V_contact.m.xx, 
            -Velocity.y / delta_V_contact.m.yy * Friction, 
            -Velocity.z / delta_V_contact.m.zz * Friction
        );
    #endif

    // Find the target normal & tangential velocities to remove
    //
    Quaternion target_V( 0, BouncingVelocity, -Velocity.y, -Velocity.z );

    // Find the impulse to remove target velocities in contact frame of reference
    // (multiplying by dV^-1 we get the impulse needed per unit velocity)
    //
    // dV = m_a^-1 + m_b^-1 
    //    + ToContact • [                           <-- ToContact == ToWorld^-1
    //            - ( r_a × I_a_world^-1 ) × r_a
    //            - ( r_b × I_b_world^-1 ) × r_b
    //      ]
    //
    // J = dV^-1 • { -( 1 + COR ) * V.x, - V.y, - V.z }
    //
    Quaternion J = delta_V_contact.Inverse () * target_V;
    
    // Use dynamic friction in case of an exceeding friction.
    //
    double J_tangential = sqrt( J.y * J.y + J.z * J.z ); // tangential component

    if ( J_tangential > J.x * Friction ) // if excceding friction
    {
        // Normalize tangential components
        J.y /= J_tangential;
        J.z /= J_tangential;

        // Recalculate impuls
        double invm = delta_V_contact.m.xx  
                    + delta_V_contact.m.xy * Friction * J.y 
                    + delta_V_contact.m.xz * Friction * J.z;
        double j_normal = BouncingVelocity / invm;

        // Reduce friction for the given amount.
        J.x = j_normal;
        J.y *= Friction * j_normal;
        J.z *= Friction * j_normal;
    }

    return J;
}
