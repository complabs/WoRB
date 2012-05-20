/**
 *  @file      PositionProjections.cpp
 *  @brief     Implementation of the position projections algorithm.
 *  @author    Mikica Kocic
 *  @version   0.19
 *  @date      2012-05-11
 *  @copyright GNU Public License.
 */

#include "WoRB.h"

using namespace WoRB;

/////////////////////////////////////////////////////////////////////////////////////////
// Resolves collisions in the system using the position projection method.
//
void CollisionResolver::PositionProjections( unsigned maxIterations, double eps )
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
        eps = 1e-2;
    }

    // Performing position projections, until there are no contacts with notable 
    // penetrations found.
    //
    for ( unsigned iteration = 0; iteration < maxIterations; ++iteration )
    {
        // Find the contact with the largest penetration
        //
        Collision* contact = FindLargestPenetration( eps );
        if ( ! contact ) {
            break; // Done, if there are no penetrations
        }

        // Activate bodies participating in the collision that are lying inactive
        // @fixme (disabled since impulse transfer should wake bodies)
        contact->ActivateInactiveBodies ();

        // Calculate and apply position/orientation jolt that resolve the penetration
        // 
        Quaternion X_jolt[2], Q_jolt[2];
        contact->PositionProjection( X_jolt, Q_jolt, Relaxation );

        // However, the resolution may have changed the penetration of other
        // bodies, so we need to update affected collision data.
        //
        RigidBody** bodies_in_this_contact = &contact->Body_A;
        for ( unsigned i = 0; i < CollisionCount; ++i )
        {
            Collision& c_aff = Collisions[i];
            RigidBody** b_aff = &c_aff.Body_A;

            for ( unsigned a = 0; a < 2; ++a )  // For each body in scanned contacts
            {
                for ( unsigned b = 0; b < 2; ++b ) // For each body in this contact
                {
                    if ( b_aff[a] && b_aff[a] == bodies_in_this_contact[b] )
                    {
                        // dX = X_j + ( Q_j x R )
                        Quaternion deltaPosition = 
                            X_jolt[b] + Q_jolt[b].Cross( c_aff.RelativePosition[a] );

                        // The sign of the change is positive if we're dealing with 
                        // body B and negative otherwise, as the position resolution
                        // should be _subtracted_.
                        //
                        double dP_n = deltaPosition.Dot( c_aff.Normal );
                        c_aff.Penetration += a ? dP_n : -dP_n;
                    }
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Position projection for the single collision
//
void Collision::PositionProjection(
        Quaternion X_jolt[2], // applied position change
        Quaternion Q_jolt[2], // applied orientation change
        double relaxation     // Position projections relaxation coefficient
    )
{
    RigidBody** Body = &Body_A;

    // We need to work out the inertia of each object in the direction
    // of the contact normal, due to angular inertia only.
    //
    double inverseTotalInertia = 0;
    QTensor inverse_I_world[2];
    double inverseAngInertia[2];

    for ( unsigned i = 0; i < 2; ++i ) 
    {
        if ( ! Body[i] ) {
            continue;
        }

        inverse_I_world[i] = Body[i]->InverseInertiaWorld;

        // Calculate the angular component of total inertia:
        // angI = ( ( I_world^-1 • ( r × N ) ) × r ) • N
        //
        inverseAngInertia[i] = 
            ( inverse_I_world[i] * RelativePosition[i].Cross( Normal ) )
            .Cross( RelativePosition[i] )
            .Dot( Normal );

        // The total inertia is sum of its linear and angular components:
        // 1/m_tot = 1/m_linear + 1/m_angular;
        //
        inverseTotalInertia += Body[i]->InverseMass + inverseAngInertia[i];
    }

    // Loop through again calculating and applying the changes
    //
    for ( unsigned i = 0; i < 2; ++i ) 
    {
        if ( ! Body[i]) {
            continue;
        }

        // Calculate required linear (delta_X) and angular movements (delta_Q) 
        // in proportion to the two mass and angular inertias
        //
        double penetration = i == 0 ? Penetration : -Penetration;
        if ( 0.0 < relaxation && relaxation <= 1.0 ) {
            penetration *= ( 1 - relaxation );
        }
        double delta_X = penetration * ( Body[i]->InverseMass / inverseTotalInertia );
        double delta_Q = penetration * ( inverseAngInertia[i] / inverseTotalInertia );

        // Limit the angular jolt to avoid angular projections that are too great 
        // (case when mass is large and inertia tensor is small).
        {
            Quaternion angularProjection = 
                RelativePosition[i] - Normal * RelativePosition[i].Dot( Normal );

            const double Q_limit = 0.3;
            double max_Q = Q_limit * angularProjection.ImNorm ();

            if ( delta_Q < -max_Q )
            {
                delta_X = ( delta_X + delta_Q ) + max_Q;
                delta_Q = -max_Q;
            }
            else if ( delta_Q > max_Q )
            {
                delta_X = ( delta_X + delta_Q ) - max_Q;
                delta_Q = max_Q;
            }
        }

        // Calculate and apply the position jolt from the required linear movement
        // delta_X, which is a simple linear movement along the contact normal.
        //
        X_jolt[i] = Normal * delta_X;
        Body[i]->Position += X_jolt[i];

        // Calculate the required angular jolt to achieve angular movement delta_Q
        //
        if ( delta_Q == 0 )  // No angular movement = no orientation jolt
        {
            Q_jolt[i]= 0;
        }
        else // a bit more complicated orientation jolt
        { 
            //                  I_world^-1 • ( r × N ) 
            // Q_jolt = ---------------------------------------- * delta_Q
            //           ( ( I_world^-1 • ( r × N ) ) × r ) • N
            //
            Q_jolt[i] = inverse_I_world[i] 
                      * RelativePosition[i].Cross( Normal ) 
                      * ( delta_Q / inverseAngInertia[i] );

            Body[i]->Orientation += 0.5 * Q_jolt[i] * Body[i]->Orientation;

            // Now, normalize orientation and recalculate transformation matrix
            //
            Body[i]->CalculateDerivedQuantities (); 
        }
    }
}
