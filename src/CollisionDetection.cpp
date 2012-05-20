/**
 *  @file      CollisionDetection.cpp
 *  @brief     Implementation of the collision detection system.
 *  @author    Mikica Kocic
 *  @version   0.18
 *  @date      2012-05-18
 *  @copyright GNU Public License.
 */

#include "WoRB.h"

#include <algorithm> // std::min, std::max

using namespace WoRB;

/////////////////////////////////////////////////////////////////////////////////////////

void Geometry::Detect( CollisionResolver& owner, const Geometry* B ) const
{
    if ( ! owner.HasSpaceForMoreContacts () ) {
        return;
    }

    typedef const Cuboid*     Cuboid_    ;
    typedef const Sphere*     Sphere_    ;
    typedef const HalfSpace*  HalfSpace_ ;
    typedef const TruePlane*  TruePlane_ ;

    switch( Class )
    {
        case _Sphere: switch( B->Class )
        {
            case _Sphere:    Sphere_(this)->Check( owner, *Sphere_(B)    ); break;
            case _Cuboid:    Cuboid_(B)   ->Check( owner, *Sphere_(this) ); break;
            case _HalfSpace: Sphere_(this)->Check( owner, *HalfSpace_(B) ); break;
            case _TruePlane: Sphere_(this)->Check( owner, *TruePlane_(B) ); break;
        }
        break;

        case _Cuboid: switch( B->Class )
        {
            case _Sphere:    Cuboid_(this)->Check( owner, *Sphere_(B)    ); break;
            case _Cuboid:    Cuboid_(this)->Check( owner, *Cuboid_(B)    ); break;
            case _HalfSpace: Cuboid_(this)->Check( owner, *HalfSpace_(B) ); break;
            case _TruePlane: /* not implemented */                        ; break;
        }
        break;

        case _HalfSpace: switch( B->Class )
        {
            case _Sphere:    Sphere_(B)->Check( owner, *HalfSpace_(this) ); break;
            case _Cuboid:    Cuboid_(B)->Check( owner, *HalfSpace_(this) ); break;
            case _HalfSpace: /* not implemented */                        ; break;
            case _TruePlane: /* not implemented */                        ; break;
        }
        break;

        case _TruePlane: switch( B->Class )
        {
            case _Sphere:    Sphere_(B)->Check( owner, *TruePlane_(this) ); break;
            case _Cuboid:    /* not implemented */                        ; break;
            case _HalfSpace: /* not implemented */                        ; break;
            case _TruePlane: /* not implemented */                        ; break;
        }
        break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Sphere::Check( CollisionResolver& owner, const TruePlane& plane ) const
{
    if ( ! owner.HasSpaceForMoreContacts () ) { 
        return 0; // We do not have space left for new contacts
    }

    // Get the position of the center of the sphere
    //
    Quaternion position = Position ();

    // Find the distance the center of the sphere to the plane
    //
    double distance = plane.Direction.Dot( position ) - plane.Offset;

    // Check if there is an overlap
    //
    if ( distance * distance > Radius * Radius )
    {
        return 0;
    }

    // Check which side of the plane we're on
    //
    Quaternion normal = plane.Direction;
    double penetration = -distance;
    if ( distance < 0 )
    {
        normal = -normal;
        penetration = -penetration;
    }
    penetration += Radius;

    return owner.RegisterNewContact(
        /* a */ Body,
        /* b */ 0, /* scenery */
        /* X */ position - plane.Direction * distance,
        /* N */ normal,
        /* d */ penetration
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Sphere::Check( CollisionResolver& owner, const HalfSpace& plane ) const
{
    if ( ! owner.HasSpaceForMoreContacts () ) { 
        return 0; // We do not have space left for new contacts
    }

    // Get the position of the center of the sphere
    //
    Quaternion position = Position ();

    // Find the distance from the plane
    //
    double distance = plane.Direction.Dot( position ) - Radius - plane.Offset;

    if ( distance >= 0 ) {
        return 0;
    }

    // New contact: it has a normal in the plane direction.
    //
    return owner.RegisterNewContact(
        /* a */ Body,
        /* b */ 0, /* scenery */
        /* X */ position - plane.Direction * ( distance + Radius ),
        /* N */ plane.Direction,
        /* d */ -distance
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Sphere::Check( CollisionResolver& owner, const Sphere& B ) const
{
    if ( ! owner.HasSpaceForMoreContacts () ) { 
        return 0; // We do not have space left for new contacts
    }

    // Get the positions of the centra of both spheres
    //
    Quaternion position_A = Position ();
    Quaternion position_B = B.Position ();

    // Find the displacement and the distance between the two spheres
    //
    Quaternion displacement = position_A - position_B;
    double distance = displacement.ImNorm ();

    // Check if the separation is large enough
    //
    if ( distance >= Radius + B.Radius) {
        return 0;
    }

    // The contact-normal is along the displacement with the position half-way
    //
    return owner.RegisterNewContact(
        /* a */ Body,
        /* b */ B.Body,
        /* X */ position_B + displacement * 0.5,
        /* N */ displacement * ( 1.0 / distance ),
        /* d */ Radius + B.Radius - distance
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

inline Quaternion Cuboid::FindContactPointOnEdges(
        const Quaternion& ptOn_A, const Quaternion& axis_A, double edge_A,
        const Quaternion& ptOn_B, const Quaternion& axis_B, double edge_B,
        bool use_A
    )
{
    // If use_A is true, and the contact point is outside the edge (in the case of 
    // an edge-face contact) then we use 'this' cuboid's midpoint, otherwise we use B's.

    double sqNorm_dA = axis_A.ImSquaredNorm(); // d_A = Axis A
    double sqNorm_dB = axis_B.ImSquaredNorm(); // d_B = Axis B
    double axis_AB = axis_B.Dot( axis_A ); // Scalar product axis_A and axis_B

    // Displacement between point on edge of this cuboid (A) and point on edge B
    //
    Quaternion p_AB = ptOn_A - ptOn_B;
    double dpSta_A = p_AB.Dot( axis_A );
    double dpSta_B = p_AB.Dot( axis_B );

    double denominator = sqNorm_dA * sqNorm_dB - axis_AB * axis_AB;

    // In case of parallel lines
    //
    if ( fabs( denominator ) < 1e-4 ) {
        return use_A ? ptOn_A : ptOn_B;
    }

    double mu_A = ( axis_AB   * dpSta_B - sqNorm_dB * dpSta_A ) / denominator;
    double mu_B = ( sqNorm_dA * dpSta_B - axis_AB   * dpSta_A ) / denominator;

    // If either of the edges has the nearest point out of bounds, 
    // then the edges aren't crossed, we have an edge-face contact. 
    //
    if ( mu_A > edge_A || mu_A < -edge_A || mu_B > edge_B || mu_B < -edge_B )
    {
        // Our point is on the edge of body which we know from the use_A parameter.
        return use_A ? ptOn_A : ptOn_B;
    }
    else
    {
        return ( ptOn_A + axis_A * mu_A ) * 0.5
             + ( ptOn_B + axis_B * mu_B ) * 0.5;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Cuboid::RegisterContactOnAxis_Thorough
(
    CollisionResolver& owner, const Cuboid& B,
    const Quaternion& displacement, // = B.Position () - Position ()
    unsigned axis
    ) const
{
    Quaternion En = Axis( axis );
    bool onRight = En.Dot( displacement ) > 0;

    // Go through each combination of +/- for each half-size
    //
    unsigned contactCount = 0;

    for ( unsigned i = 0; i < 8 && owner.HasSpaceForMoreContacts (); ++i )
    {
        // Calculate the position of each vertex
        //
        static const double vertices[8][3] = 
        {
            { 1, 1,  1 }, { -1, 1,  1 }, { 1, -1,  1 }, { -1, -1,  1 },
            { 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, { -1, -1, -1 }
        };
        Quaternion vertexPos( 0, vertices[i][0], vertices[i][1], vertices[i][2] );
        vertexPos = vertexPos.ComponentWiseProduct( B.HalfExtent );
        vertexPos = B.Body->ToWorld( vertexPos );

        // Calculate the distance between the B's vertex and the A's center
        // projected on A's axis
        //
        double distance = ( vertexPos - Position() ).Dot( En );

        // Compare this to the A's face distance from A's center
        //
        if ( ( onRight && distance <= HalfExtent[ axis ] )
          || ( !onRight && distance >= -HalfExtent[ axis ] ) )
        {
            // New contact with the vertex as the point of contact
            //
            contactCount += owner.RegisterNewContact(
                /* a */ Body,
                /* b */ B.Body,
                /* X */ vertexPos,
                /* N */ onRight ? -En : En,
                /* d */ onRight ? HalfExtent[ axis ] - distance
                                : distance - HalfExtent[ axis ]
            );
        }
    }

    return contactCount;
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Cuboid::RegisterContactOnAxis
(
    CollisionResolver& owner, const Cuboid& B,
    const Quaternion& displacement,
    const Quaternion& axis,
    double penetration
    ) const
{
    // Find out which of the B faces is on the specified axis.
    //
    Quaternion normal = axis;
    if ( normal.Dot( displacement ) > 0 ) {
        normal = -normal;
    }

    Quaternion axis_Bn( 0,
        B.Axis(0).Dot( normal ),
        B.Axis(1).Dot( normal ),
        B.Axis(2).Dot( normal )
    );

    // Find out which vertex of cuboid B we are colliding with.

    Quaternion contactPointOn_B; // = zero

    for ( unsigned i = 0; i < 3; ++i ) // for each axis in B
    {
        // In case of B's edge that is almost normal to the contact normal 
        // we take a mid-point of A's & B's edges intersection projected on B.Axis(i).
        // Otherwise, if not normal, we take the vertex which is the closest.
        //
        if ( fabs( axis_Bn[i] ) < 1e-4 )
        {
            // Project the A's half-extents and displacement onto B's axis
            // deltaCenter +/- halfExtent_A describes A's vertices in B's frame of ref.
            //
            double distance_BA  = -displacement.Dot( B.Axis(i) );
            double halfExtent_A = ProjectOn( B.Axis(i) );
            double halfExtent_B = B.HalfExtent[i];

            // Get mid-point of A's & B's projections' intersection on B's axis
            //
            double vxL = std::max( distance_BA - halfExtent_A, -halfExtent_B );
            double vxR = std::min( distance_BA + halfExtent_A,  halfExtent_B );
            double vxM = 0.5 * ( vxL + vxR );
            contactPointOn_B[i] = fabs( vxM ) < 1e-4 ? 0 : vxM;
        }
        else // if B's edge is not almost normal to the contact normal 
        {
            // Select the closest vertex
            //
            contactPointOn_B[i] = axis_Bn[i] > 0 ?  B.HalfExtent[i]
                                                 : -B.HalfExtent[i];
        }
    }

    // No parallel axes with faces found; single collision point
    //
    return owner.RegisterNewContact(
        /* a */ Body,
        /* b */ B.Body,
        /* X */ B.Body->ToWorld( contactPointOn_B ),
        /* N */ normal,
        /* d */ penetration
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Cuboid::Check( CollisionResolver& owner, const Cuboid& B ) const
{
    // Find the displacement between the two centra
    //
    Quaternion displacement = B.Position() - Position();

    // Assume that there is no contact
    //
    double penetration = Const::Max;
    unsigned int axisIndex_A = 0xFF;
    unsigned int axisIndex_B = 0xFF;

    #define Quit_If_No_Overlaps( axis, indexA, indexB ) \
    if ( ! CheckOverlapOnAxis( B, (axis), displacement, penetration, \
            (indexA), (indexB), axisIndex_A, axisIndex_B ) ) { \
        return 0; \
    }

    // Check each body axes, keeping track of the axis with the smallest penetration
    //
    Quit_If_No_Overlaps( Axis(0)  ,    0, 0xFF );
    Quit_If_No_Overlaps( Axis(1)  ,    1, 0xFF );
    Quit_If_No_Overlaps( Axis(2)  ,    2, 0xFF );
    Quit_If_No_Overlaps( B.Axis(0), 0xFF,    0 );
    Quit_If_No_Overlaps( B.Axis(1), 0xFF,    1 );
    Quit_If_No_Overlaps( B.Axis(2), 0xFF,    2 );

    // Remember the single axis with the smallest penetration
    //
    bool use_A = axisIndex_B != 0xFF;

    // Continue with the axes cross products
    //
    Quit_If_No_Overlaps( Axis(0).Cross( B.Axis(0) ), 0, 0 );
    Quit_If_No_Overlaps( Axis(0).Cross( B.Axis(1) ), 0, 1 );
    Quit_If_No_Overlaps( Axis(0).Cross( B.Axis(2) ), 0, 2 );
    Quit_If_No_Overlaps( Axis(1).Cross( B.Axis(0) ), 1, 0 );
    Quit_If_No_Overlaps( Axis(1).Cross( B.Axis(1) ), 1, 1 );
    Quit_If_No_Overlaps( Axis(1).Cross( B.Axis(2) ), 1, 2 );
    Quit_If_No_Overlaps( Axis(2).Cross( B.Axis(0) ), 2, 0 );
    Quit_If_No_Overlaps( Axis(2).Cross( B.Axis(1) ), 2, 1 );
    Quit_If_No_Overlaps( Axis(2).Cross( B.Axis(2) ), 2, 2 );
                                                           
    #undef Quit_If_No_Overlaps

    // At this point we have detected a collision!

    if ( axisIndex_B == 0xFF ) // single axis A
    {
        // The collision between cuboid B's vertex and this cuboid's face
        //
        return this->RegisterContactOnAxis( owner, B, 
            displacement, Axis( axisIndex_A ), penetration );
    }
    else if ( axisIndex_A == 0xFF ) // single axis B
    {
        // The collision between this cuboid's vertex and cuboid B's face
        //
        return B.RegisterContactOnAxis( owner, *this, 
            -displacement, B.Axis( axisIndex_B ), penetration );
    }

    // It is an edge-edge collision; get the involved axes from indices
    //
    Quaternion axis_A = Axis( axisIndex_A );
    Quaternion axis_B = B.Axis( axisIndex_B );

    Quaternion normal = axis_A.Cross( axis_B ).Unit ();

    // Make the normal always point from A to B
    //
    if ( normal.Dot( displacement ) > 0 ) {
        normal = -normal;
    }

    // Find the closest point on the involved edges to the other axes.
    // By default the closest point is mid-point on the edge.
    // @note  Compare to RegisterContactOnAxis algorithm.
    //
    Quaternion ptOn_Edge_A; // = zero (mid-point on all edges by default)
    Quaternion ptOn_Edge_B; // = zero (mid-point on all edges by default)

    for ( unsigned i = 0; i < 3; ++i ) // foreach axis in A and B
    {
        if ( i != axisIndex_A )
        {
            double axis_An = Axis(i).Dot( normal );

            if ( fabs( axis_An ) > 1e-4 ) {
                ptOn_Edge_A[i] = axis_An > 0 ? -HalfExtent[i] 
                                             :  HalfExtent[i];
            }
        }
        if ( i != axisIndex_B )
        {
            double axis_Bn = B.Axis(i).Dot( normal );

            if ( fabs( axis_Bn ) > 1e-4 ) {
                ptOn_Edge_B[i] = axis_Bn > 0 ?  B.HalfExtent[i] 
                                             : -B.HalfExtent[i];
            }
        }
    }

    // At this moment we have a point and a direction for the colliding edges.
    // Now, find out the contact point i.e. the closest approach of B's line-segments.
    // (Note that, from here, we continue in world coordinates.)
    //
    Quaternion contactPoint_world = FindContactPointOnEdges(
        Body->ToWorld( ptOn_Edge_A ),   axis_A, HalfExtent[ axisIndex_A ],
        B.Body->ToWorld( ptOn_Edge_B ), axis_B, B.HalfExtent[ axisIndex_B ], use_A
    );

    return owner.RegisterNewContact(
        /* a */ Body,
        /* b */ B.Body,
        /* X */ contactPoint_world,
        /* N */ normal,
        /* d */ penetration
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Cuboid::Check( CollisionResolver& owner, const Quaternion& point ) const
{
    // Transform the point into cuboid coordinates i.e. body-fixed frame
    //
    Quaternion pointInBodySpace = Body->ToWorld.TransformInverse( point );

    Quaternion normal;
    double min_depth = Const::Max;

    // Check each axis, looking for the axis on which the penetration is least deep.
    //
    for ( unsigned i = 0; i < 3; ++i )
    {
        double depth = HalfExtent[i] - fabs( pointInBodySpace[i] );
        if ( depth < 0 ) {
            return 0;
        }
        else if ( depth < min_depth )
        {
            min_depth = depth;
            normal = pointInBodySpace[i] < 0 ? -Axis(i) : Axis(i);
        }
    }

    return owner.RegisterNewContact(
        /* a */ Body,
        /* b */ 0, // == scenery
        /* X */ point,
        /* N */ normal,
        /* d */ min_depth
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Cuboid::Check( CollisionResolver& owner, const Sphere& B ) const
{
    // Transform the center of the sphere into cuboid coordinates
    //
    Quaternion center = B.Position ();
    Quaternion relCenter = Body->ToWorld.TransformInverse( center );

    // Early out check to see if we can exclude the contact
    //
    if ( fabs( relCenter.x ) - B.Radius > HalfExtent.x ||
         fabs( relCenter.y ) - B.Radius > HalfExtent.y ||
         fabs( relCenter.z ) - B.Radius > HalfExtent.z )
    {
        return 0;
    }

    // Clamp each coordinate to the cuboid
    //
    Quaternion closestPoint( 0,
        Clamp( relCenter.x, HalfExtent.x ),
        Clamp( relCenter.y, HalfExtent.y ),
        Clamp( relCenter.z, HalfExtent.z )
    );

    // Check we're in contact
    //
    double distance = ( closestPoint - relCenter ).ImSquaredNorm();
    if ( distance > B.Radius * B.Radius ) {
        return 0;
    }
    distance = sqrt( distance );

    // New contact at the closest point in world coordinates
    //
    Quaternion closestPointWorld = Body->ToWorld( closestPoint );

    return owner.RegisterNewContact(
        /* a */ Body,
        /* b */ B.Body,
        /* X */ closestPointWorld,
        /* N */ ( closestPointWorld - center ).Unit (),
        /* d */ B.Radius - distance
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

unsigned Cuboid::Check( CollisionResolver& owner, const HalfSpace& plane ) const
{
    if ( ! owner.HasSpaceForMoreContacts () ) { 
        return 0; // We do not have space left for new contacts
    }
    else if ( ! Intersects( plane ) ) {
        return 0; // No intersection between the cuboid and the half-space
    }

    // The contact point with the largest penetration
    // By default zero, meaning the mid-point on all edges of the cuboid.
    //
    Quaternion contactPoint; 

    // Find if there exist edges/faces of the cuboid parallel to the plane

    Quaternion axis_n( 0,
        Axis(0).Dot( plane.Direction ),
        Axis(1).Dot( plane.Direction ),
        Axis(2).Dot( plane.Direction )
    );

    unsigned parallelCount = 0;

    for ( unsigned i = 0; i < 3; ++i ) // for each axis in B
    {
        // In case of cuboid's edge that is almost parallel to the plane 
        // we take a mid-point of the edge.
        // Otherwise, if parallel, we take the vertex which is the closest.
        //
        if ( fabs( axis_n[i] ) < 1e-4 ) // almost parallel edge & plane
        {
            ++parallelCount;
        }
        else // if cuboid's edge is not almost parallel to the plane
        {
            // Select the closest vertex
            //
            contactPoint[i] = axis_n[i] < 0 ?  HalfExtent[i]
                                            : -HalfExtent[i];
        }
    }

    // In case if there are edges/faces parallel to the plane,
    // just register the found mid-contact point and return.
    //
    if ( parallelCount > 0 )
    {
        // Switch to world coordinates and find out penetration of the
        // contact point into the half-space
        //
        contactPoint = Body->ToWorld( contactPoint );
        double penetration = plane.Offset - contactPoint.Dot( plane.Direction );

        // Register a new contact at the point of contact
        //
        return owner.RegisterNewContact(
            /* a */ Body,
            /* b */ 0, // scenery
            /* X */ contactPoint + 0.5 * penetration * plane.Direction,
            /* N */ plane.Direction,
            /* d */ penetration
        );
    }

    // Proceede with the thorough investigation by scanning all vertices

    // Find the intersection points by checking all vertices. 
    // If the cuboid is resting on a plane or on an edge, it will be reported 
    // as four or two contact points.

    // Go through each combination of +/- for each half-size
    //
    unsigned contactCount = 0;

    for ( unsigned i = 0; i < 8 && owner.HasSpaceForMoreContacts (); ++i )
    {
        // Calculate the position of each vertex
        //
        static const double vertices[8][3] = 
        {
            { 1, 1,  1 }, { -1, 1,  1 }, { 1, -1,  1 }, { -1, -1,  1 },
            { 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, { -1, -1, -1 }
        };
        Quaternion vertexPos( 0, vertices[i][0], vertices[i][1], vertices[i][2] );

        vertexPos = vertexPos.ComponentWiseProduct( HalfExtent );

        vertexPos = Body->ToWorld( vertexPos );

        // Calculate the penetration of the vertex into the half-space
        //
        double penetration = plane.Offset - vertexPos.Dot( plane.Direction );

        if ( penetration >= 0 ) // In case of penetration...
        {
            // Register a new contact where the point of contact is half-way between 
            // the vertex and the plane
            //
            contactCount += owner.RegisterNewContact(
                /* a */ Body,
                /* b */ 0, // scenery
                /* X */ vertexPos + 0.5 * penetration * plane.Direction,
                /* N */ plane.Direction,
                /* d */ penetration
            );
        }
    }

    return contactCount;
}
