#ifndef _WORB_GEOMETRY_H_INCLUDED
#define _WORB_GEOMETRY_H_INCLUDED

/**
 *  @file      Geometry.h
 *  @brief     Definitions for the Geometry and its derived classes (Sphere, Cuboid etc)
 *             which implement collision detection ('space awereness') algorithms.
 *  @author    Mikica Kocic
 *  @version   0.19
 *  @date      2012-05-12
 *  @copyright GNU Public License.
 */

#include "Constants.h"

namespace WoRB 
{
    class CollisionResolver;

    void Printf( const char* format, ... );

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents a geometry used to detect collisions against.
     * Cannot be explicitly instantiated.
     */
    class Geometry
    {
    protected:

        enum GeometryClass
        {
            _Sphere,
            _Cuboid,
            _HalfSpace,
            _TruePlane
        };

        /** Holds the geometry class of the object.
         */
        GeometryClass Class;

        /** Protected constructor; disallows explicit instantiation of the class.
         */
        Geometry( GeometryClass type, RigidBody* body = 0 )
            : Class( type )
            , Body( body )
        {
        }

    public:

        bool IsCuboid ()    const { return Class == _Cuboid;    }
        bool IsSphere ()    const { return Class == _Sphere;    }
        bool IsHalfSpace () const { return Class == _HalfSpace; }
        bool IsTruePlane () const { return Class == _TruePlane; }

        /** Returns class of the geometry as a string.
         */
        const char* GetName () const
        {
            switch( Class )
            {
                case _Sphere:    return "Sphere";
                case _Cuboid:    return "Cuboid";
                case _HalfSpace: return "HalfSpace";
                case _TruePlane: return "TruePlane";
            }
            return "(unknown)";
        }

        /** The rigid body that is represented by this geometry.
         * Null in case of a scenery object (like a half-plane).
         */
        RigidBody* Body;

        /** Gets the position vector of the geometry.
         */
        Quaternion Position () const
        {
            // Column with index 3 holds the position
            //
            return Body ? Body->ToWorld.Column(3) : 0.0; 
        }

        /** Gets the unit base vector (axes) of the geometry, given by the index.
         */
        Quaternion Axis( unsigned index ) const
        {
            // Axes are columns of the q-tensor with indices 0-2
            //
            return Body ? Body->ToWorld.Column( index ) : 0.0;
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Detects and registers a collision between this and the other geometry
         */
        void Detect( CollisionResolver& owner, const Geometry* B ) const;
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents a half-space defined by a plane where the normal of the plane 
     * points out of the half-space.
     */
    class HalfSpace : public Geometry
    {
    public:

        HalfSpace ()
            : Geometry( Geometry::_HalfSpace )
            , Offset( 0 )
        {
        }

        /** The plane normal
         */
        Quaternion Direction;

        /** The distance of the plane from the origin.
         */
        double Offset;
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents a true plane.
     */
    class TruePlane : public Geometry
    {
    public:

        TruePlane ()
            : Geometry( Geometry::_TruePlane )
            , Offset( 0 )
        {
        }

        /** The plane normal
         */
        Quaternion Direction;

        /** The distance of the plane from the origin.
         */
        double Offset;
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Encapsulates a sphere.
     */
    class Sphere : public Geometry
    {
    public:

        Sphere ()
            : Geometry( Geometry::_Sphere )
            , Radius( 0 )
        {
        }

        /** Holds the radius of the sphere.
         */
        double Radius;

        /** Gets the volume of the sphere.
         */
        double Volume () const
        {
            return ( 4.0/3.0 * Const::Pi ) * Radius * Radius * Radius;
        }

        /** Sets body mass and principal moment of inertia of the sphere.
         */
        void SetMass( double mass )
        {
            Body->SetupMass( mass );

            double Ixx = (2.0/5.0) * mass * Radius * Radius;
            Body->SetMomentOfInertia( QTensor( Ixx, Ixx, Ixx ) );

            Body->CalculateDerivedQuantities( /*fromMomenta*/ false );
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Tests for intersection of the sphere and a half-space.
         */
        bool Intersects( const HalfSpace& plane ) const
        {
            // Find the distance from the origin
            double distance = plane.Direction.Dot( Position() ) - Radius;

            // Check for the intersection
            return distance <= plane.Offset;
        }

        /** Tests for intersection between two spheres.
         */
        bool Intersects( const Sphere& B ) const
        {
            // Find the vector between the objects
            Quaternion displacement = Position() - B.Position();

            // See if it is large enough.
            double sumRadius = Radius + B.Radius;
            return displacement.ImSquaredNorm() < sumRadius * sumRadius;
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Checks for collision between the sphere and a half-space.
         */
        unsigned Check( CollisionResolver& owner, const HalfSpace& plane ) const;

        /** Checks for collision between the sphere and a true plane.
         */
        unsigned Check( CollisionResolver& owner, const TruePlane& plane ) const;

        /** Checks for collision between two spheres.
         */
        unsigned Check( CollisionResolver& owner, const Sphere& B ) const;
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Encapsulates a cuboid (rectangular parallelepiped).
     */
    class Cuboid : public Geometry
    {
    public:

        Cuboid ()
            : Geometry( Geometry::_Cuboid )
        {
        }

        /** Holds the half-extent of the cuboid along each of its local axes.
         */
        Quaternion HalfExtent;

        /** Gets the volume of the cuboid.
         */
        double Volume () const
        {
            return 8.0 * HalfExtent.x * HalfExtent.y * HalfExtent.z;
        }

        /** Sets body mass and principal moment of inertia of the cuboid.
         */
        void SetMass( double mass )
        {
            Body->SetupMass( mass );

            Quaternion extent = 2.0 * HalfExtent;
            Quaternion sq = extent.ComponentWiseProduct( extent );

            Body->SetMomentOfInertia( QTensor(
                mass * ( sq.y + sq.z ) / 12,
                mass * ( sq.x + sq.z ) / 12,
                mass * ( sq.x + sq.y ) / 12
            ) );

            Body->CalculateDerivedQuantities( /*fromMomenta*/ false );
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Tests for intersection of the cuboid and a half-space.
         */
        bool Intersects( const HalfSpace& plane ) const
        {
            // Calculate the projected radius of the cuboid onto the plane direction
            double projectedRadius = ProjectOn( plane.Direction );

            // Calculate how far the cuboid is from the origin
            double distance = plane.Direction.Dot( Position() ) - projectedRadius;

            // Check for the intersection
            return distance <= plane.Offset;
        }

        /** Tests for intersection between this and some other cuboid.
         */
        bool Intersects( const Cuboid& B ) const
        {
            // Find the displacement between the centra of two cuboids
            //
            Quaternion displacement = B.Position() - Position();

            // Now, first, check axes of A, then axes of B, and finally their cross products
            //
            return IsOverlapOnAxis( B, Axis(0),                    displacement )
                && IsOverlapOnAxis( B, Axis(1),                    displacement )
                && IsOverlapOnAxis( B, Axis(2),                    displacement )
                && IsOverlapOnAxis( B,                B.Axis(0),   displacement )
                && IsOverlapOnAxis( B,                B.Axis(1),   displacement )
                && IsOverlapOnAxis( B,                B.Axis(2),   displacement )
                && IsOverlapOnAxis( B, Axis(0).Cross( B.Axis(0) ), displacement )
                && IsOverlapOnAxis( B, Axis(0).Cross( B.Axis(1) ), displacement )
                && IsOverlapOnAxis( B, Axis(0).Cross( B.Axis(2) ), displacement )
                && IsOverlapOnAxis( B, Axis(1).Cross( B.Axis(0) ), displacement )
                && IsOverlapOnAxis( B, Axis(1).Cross( B.Axis(1) ), displacement )
                && IsOverlapOnAxis( B, Axis(1).Cross( B.Axis(2) ), displacement )
                && IsOverlapOnAxis( B, Axis(2).Cross( B.Axis(0) ), displacement )
                && IsOverlapOnAxis( B, Axis(2).Cross( B.Axis(1) ), displacement )
                && IsOverlapOnAxis( B, Axis(2).Cross( B.Axis(2) ), displacement );
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Checks for collision between the cuboid and a half-space.
         */
        unsigned Check( CollisionResolver& owner, const HalfSpace& plane ) const;

        /** Checks for collision between the cuboid and a point.
         */
        unsigned Check( CollisionResolver& owner, const Quaternion& point ) const;

        /** Checks for collision between this and some other cuboid.
         */
        unsigned Check( CollisionResolver& owner, const Cuboid& B ) const;

        /** Checks for collision between the cuboid and a sphere.
         */
        unsigned Check( CollisionResolver& owner, const Sphere& B ) const;

        /////////////////////////////////////////////////////////////////////////////////

    private:

        /////////////////////////////////////////////////////////////////////////////////
        /** @name Private methods: Intersection detection related                      */
                                                                                   /*@{*/
        /** Clamps the given value to some +/- max value.
         */
        static inline double Clamp( double x, double max ) {
            return x > max ? max : x < -max ? -max : x;
        }

        /** Gets the sum of cuboid's half-extent's projections on the given vector.
         */
        double ProjectOn( const Quaternion& vector ) const
        {
            return HalfExtent.x * fabs( vector.Dot( Axis(0) ) )
                 + HalfExtent.y * fabs( vector.Dot( Axis(1) ) )
                 + HalfExtent.z * fabs( vector.Dot( Axis(2) ) );
        }

        /** Checks if this and another cuboid overlap along the given direction. 
         * @return The ammount of penetration, where positive value indicates overlap.
         */
        double GetPenetrationOnAxis( const Cuboid& B,
            const Quaternion& axis, const Quaternion& displacement
            ) const
        {
            Quaternion direction = axis.Unit ();

            // Project the half-extents and displacement onto axis
            double proj_A = ProjectOn( direction );
            double proj_B = B.ProjectOn( direction );
            double distance = fabs( displacement.Dot( direction ) );

            // Return the overlap 
            return proj_A + proj_B - distance;
        }

        /** Checks if this and another cuboid overlap along the given direction.
         */
        bool IsOverlapOnAxis( const Cuboid& B,
            const Quaternion& direction, const Quaternion& displacement
            ) const
        {
            // Skip almost parallel axes.
            if ( direction.ImSquaredNorm() < 1e-4 ) {
                return true;
            }

            return GetPenetrationOnAxis( B, direction, displacement ) > 0;
        }

        /** Checks for overlap along the given direction.
         * Keeps track of the smallest penetration.
         */
        bool CheckOverlapOnAxis( const Cuboid& B,
            const Quaternion& direction, const Quaternion& displacement, 
            double& smallestPenetration, 
            unsigned tag_A, unsigned tag_B, 
            unsigned& indexTag_A, unsigned& indexTag_B
            ) const
        {
            // Skip almost parallel axes.
            if ( direction.ImSquaredNorm() < 1e-4 ) {
                return true;
            }

            // Get penetration depth on axis.
            double penetration = GetPenetrationOnAxis( B, direction, displacement );

            if ( penetration < 0 ) { // no penetration
                return false;
            }
            else if ( penetration < smallestPenetration /*- 1e-6 */ ) {
                smallestPenetration = penetration;
                indexTag_A = tag_A;
                indexTag_B = tag_B;
            }
            return true;
        }

        /** Finds point of contact between two points on two cuboid edges.
         */
        static Quaternion FindContactPointOnEdges(
            const  Quaternion& pt_A,   //!< Point on the edge of the cuboid A
            const  Quaternion& axis_A, //!< Axis of the cuboid A
            double size_A,             //!< Half-extent of A's axis
            const  Quaternion& pt_B,   //!< Point on the edge of the cuboid B
            const  Quaternion& axis_B, //!< Axis of the cuboid B
            double size_B,             //!< Half-extent of B's axis
            bool   use_A               //!< Which cuboid to use, if point is on the edge
            );

        /** Registers a contact between this and the other cuboid body along an axis.
         * @return 1 if ok, 0 if failed to allocate space for the contact.
         */
        unsigned RegisterContactOnAxis_Thorough
        (
            CollisionResolver& owner,       //!< The collision registry
            const Cuboid& B,                //!< The second cuboid
            const Quaternion& displacement, //!< The displacement between B and A centra
            unsigned axis                   //!< A's axis
            ) const;

        /** Registers a contact between this and the other cuboid body along an axis.
         * Faster variant that does not check every vertex but the closest one.
         * @return 1 if ok, 0 if failed to allocate space for the contact.
         */
        unsigned RegisterContactOnAxis
        (
            CollisionResolver& owner,       //!< The collision registry
            const Cuboid& B,                //!< The second cuboid
            const Quaternion& displacement, //!< The displacement between B and A centra
            const Quaternion& axis,         //!< The axis of this couboid (cuboid A)
            double penetration              //!< The penetration depth
            ) const;
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
    };

} // namespace WoRB

#endif // _WORB_GEOMETRY_H_INCLUDED
