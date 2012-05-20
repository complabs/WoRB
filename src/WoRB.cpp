/**
 *  @file      WoRB.cpp
 *  @brief     Implementation of miscellaneous methods for some of WoRB classes.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-05-03
 *  @copyright GNU Public License.
 */

#include "WoRB.h"

#include <cmath>  // we use atan
#include <limits> // we use std::numeric_limits

using namespace WoRB;

/////////////////////////////////////////////////////////////////////////////////////////

void Quaternion::Dump ( const char* name ) const
{
    Printf( "%10s : %12.4lf %12.4lf %12.4lf | %12.4lf\n", name, x, y, z, w );
}

void Collision::Dump( unsigned id, double currentTime ) const
{
    Printf( "\nCollision %d: (COR = %g, mu = %g)\n", id, 
        Restitution, Friction );

    Printf( "%10s : %12.4lf\n", "t", currentTime );

    Position.Dump( "X" );
    Normal.Dump( "N" );

    Printf( "%10s : %12.4lf\n", "Pen", Penetration );

    Velocity.Dump( "V" );
    RelativePosition[0].Dump( "X rel A" );
    RelativePosition[1].Dump( "X rel B" );

    Printf( "%10s : %12.4lf\n", "B-Vel", BouncingVelocity );
}

void CollisionResolver::Dump( double currentTime ) const
{
    for ( unsigned i = 0; i < CollisionCount; ++i )
    {
        Collisions[i].Dump( i, currentTime );
    }
}


