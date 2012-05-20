#ifndef _QUATERNION_H_INCLUDED
#define _QUATERNION_H_INCLUDED

/**
 *  @file      Quaternion.h
 *  @brief     Definitions for the Quaternion class that encapsulates Hamilton real
 *             quaternions.
 *  @author    Mikica Kocic
 *  @version   0.5
 *  @date      2012-05-11
 *  @copyright GNU Public License.
 */

#include <cmath>  // we use: sqrt (), fabs ()

namespace WoRB 
{
    /////////////////////////////////////////////////////////////////////////////////////
    /** @class Quaternion
     *
     * Encapsulates Hamilton real quaternions.
     *
     * @note The Quaternion class is implemented with inline methods only.
     */
    class Quaternion
    {
    public:
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Quaternion components
         *
         *  - *w*: holds the real (aka scalar) part of the quaternion.
         *  - *x*, *y*, *z*: holds the pure imaginary (vector) part of the quaternion.
         *
         * See <a href="http://en.wikipedia.org/wiki/Quaternion">
         *     Quaternions on Wikipedia </a>
         */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        double w;   //!< Holds the real (scalar) part of the quaternion.
        double x;   //!< Holds _i_-component of the pure imaginary (vector) part.
        double y;   //!< Holds _j_-component of the pure imaginary (vector) part.
        double z;   //!< Holds _k_-component of the pure imaginary (vector) part.
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Constructors and assignment operators                                */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** The default constructor; creates a zero quaternion
         */
        Quaternion () 
            : w(0), x(0), y(0), z(0) 
        {
        }

        /** Creates a quaternion having only the scalar (real) part.
         */
        Quaternion( double scalar ) 
            : w(scalar), x(0), y(0), z(0) 
        {
        }

        /** Creates a quaternion with the given components.
         */
        Quaternion( double w, double x, double y, double z )
            : w(w), x(x), y(y), z(z)
        {
        }

        /** Sets the scalar (real) part of the quaternion also clearing the vector part.
          */
        Quaternion& operator = ( double scalar )
        {
            w = scalar;
            x = y = z = 0;
            return *this;
        }

        /** Constructs a quaternion from axis angle representation of rotation.
         */
        static Quaternion FromAxisAngle( double angle, 
            double vx, double vy, double vz )
        {
            double norm = sqrt( vx * vx + vy * vy + vz * vz );

            if ( norm == 0 ) {
                return Quaternion( 1 );
            }

            double Re = cos( angle/2 );
            double Im = sin( angle/2 ) / norm;

            return Quaternion( Re, Im * vx, Im * vy, Im * vz );
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Indexing operators                                                   */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        double operator [] ( unsigned index ) const
        {
            if ( index <= 2 ) {
                return *(&x + index);
            }
            return w;
        }

        double& operator [] ( unsigned index )
        {
            if ( index <= 2 ) {
                return *(&x + index);
            }
            return w;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Methods acting on quaterinion components                             */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Normalizes the quaternion to a given length.
         */
        Quaternion& Normalize( double length = 1.0 )
        {
            double norm = Norm ();

            // Handle zero length quaternion. 
            // Note that norm == 0 if and only if w == x == y == z == 0
            //
            if ( norm == 0 ) {
                w = length;   
                return *this;
            }

            return *this *= length / norm;
        }

        /** Limits the size of the quaternion to the given maximum. 
         */
        Quaternion& Trim( double size )
        {
            if ( ImSquaredNorm () > size * size )
            {
                return Normalize( size );
            }

            return *this;
        }

        /** Removes the quaternion components that are bellow the limit.
         */
        Quaternion& Zeroize( double eps = 1e-4 )
        {
            if ( fabs( x ) < eps ) x = 0;
            if ( fabs( y ) < eps ) y = 0;
            if ( fabs( z ) < eps ) z = 0;
            if ( fabs( w ) < eps ) w = 0;

            return *this;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Unary operations                                                     */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Flips signs of all the components of the quaternion. 
          */
        Quaternion operator - () const
        {
            return Quaternion( -w, -x, -y, -z );
        }

        /** Returns the conjugate of the quaternion.
          */
        Quaternion Conjugate () const
        {
            return Quaternion( w, -x, -y, -z );
        }

        /** Gets the squared magnitude of this quaternion. 
         */
        double SquaredNorm () const
        {
            return w * w + x * x + y * y + z * z;
        }

        /** Gets the squared magnitude of the scalar part of this quaternion. 
         */
        double ReSquaredNorm () const
        {
            return w * w;
        }

        /** Gets the squared magnitude of the vector part of this quaternion. 
         */
        double ImSquaredNorm () const
        {
            return x * x + y * y + z * z;
        }

        /** Gets the magnitude of this quaternion. 
         */
        double Norm () const
        {
            return sqrt( SquaredNorm () );
        }

        /** Gets the magnitude of the vector part of this quaternion. 
         */
        double ImNorm () const
        {
            return sqrt( ImSquaredNorm () );
        }

        /** Returns the normalized version (versor) of a quaternion. 
          */
        Quaternion Unit( double length = 1.0 ) const
        {
            return Quaternion( *this ).Normalize( length );
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Binary operations                                                    */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Returns the value of the given quaternion added to this.
         */
        Quaternion operator + ( const Quaternion& q ) const
        {
            return Quaternion( w + q.w, x + q.x, y + q.y, z + q.z );
        }

        /** Adds the given quaternion to this. 
         */
        Quaternion& operator += ( const Quaternion& q )
        {
            w += q.w;  x += q.x;  y += q.y;  z += q.z;
            return *this;
        }

        /** Returns the value of the given quaternion subtracted from this.
         */
        Quaternion operator - (const Quaternion& q) const
        {
            return Quaternion( w - q.w, x - q.x, y - q.y, z - q.z );
        }

        /** Subtracts the given quaternion from this. 
          */
        Quaternion operator -= ( const Quaternion& q )
        {
            w -= q.w;  x -= q.x;  y -= q.y;  z -= q.z;
            return *this;
        }

        /** Multiplies the quaternion by the given quaternion.
         */
        Quaternion operator * ( const Quaternion& q ) const
        {
            return Quaternion(
                w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y + y * q.w + z * q.x - x * q.z,
                w * q.z + z * q.w + x * q.y - y * q.x
                );
        }

        /** Multiplies the quaternion by the given quaternion.
         */
        Quaternion& operator *= ( const Quaternion& right )
        {
            return *this = *this * right;
        }

        /** Returns a copy of this quaternion scaled the given scalar value. 
         */
        Quaternion operator * ( double value ) const
        {
            return Quaternion( w * value, x * value, y * value, z * value);
        }

        /** Multiplies this quaternion by the given scalar value. 
         */
        Quaternion& operator *= ( double value )
        {
            w *= value;  x *= value;  y *= value;  z *= value;
            return *this;
        }

        /** Returns a component-wise product of this and another quaternion.
         */
        Quaternion ComponentWiseProduct( const Quaternion& q ) const
        {
            return Quaternion( w * q.w, x * q.x, y * q.y, z * q.z);
        }

        /** Returns a vector product of the vector parts of this and another quaternion.
         */
        Quaternion Cross( const Quaternion& q ) const
        {
            return Quaternion( 0,
                y * q.z - z * q.y,
                z * q.x - x * q.z,
                x * q.y - y * q.x );
        }

        /** Returns a scalar product of the vector parts of this and another quaternion.
         */
        double Dot( const Quaternion &q ) const
        {
            return /* w * q.w + */ x * q.x + y * q.y + z * q.z;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Comparison operators                                                 */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Checks if the two quaternions have identical components. 
          */
        bool operator == ( const Quaternion& q ) const
        {
            return w == q.w && x == q.x && y == q.y && z == q.z;
        }

        /** Checks if the two vectors have non-identical components. 
          */
        bool operator != ( const Quaternion& q ) const
        {
            return !( *this == q );
        }

        /** Checks if this quaternion is component-by-component less than the other.
         */
        bool operator < ( const Quaternion& q ) const
        {
            return w < q.w && x < q.x && y < q.y && z < q.z;
        }

        /** Checks if this quaternion is component-by-component less than the other.
         */
        bool operator > (const Quaternion& q) const
        {
            return w > q.w && x > q.x && y > q.y && z > q.z;
        }

        /** Checks if this quaternion is component-by-component less than the other.
         */
        bool operator <= ( const Quaternion& q ) const
        {
            return w <= q.w && x <= q.x && y <= q.y && z <= q.z;
        }

        /** Checks if this quaternion is component-by-component less than the other.
         */
        bool operator >= ( const Quaternion& q ) const
        {
            return w >= q.w && x >= q.x && y >= q.y && z >= q.z;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////

        public: void Dump ( const char* name ) const;
    };

    inline static Quaternion operator + ( double scalar, const Quaternion& q )
    {
        return Quaternion( scalar + q.w, scalar + q.x, scalar + q.y, scalar + q.z );
    }

    inline static Quaternion operator - ( double scalar, const Quaternion& q )
    {
        return Quaternion( scalar - q.w, scalar - q.x, scalar - q.y, scalar - q.z );
    }

    inline static Quaternion operator * ( double scalar, const Quaternion& q )
    {
        return Quaternion( scalar * q.w, scalar * q.x, scalar * q.y, scalar * q.z );
    }

    /////////////////////////////////////////////////////////////////////////////////////
    /** @class SpatialVector
     *
     * Encapsulates a spatial vector.
     * Spatial vector is a pure imaginary quaternion (with the imaginary part only).
     */
    class SpatialVector : public Quaternion
    {
    public:

        /** Creates a spatial vector with the given components
         */
        SpatialVector( double x, double y, double z )
            : Quaternion( 0, x, y, z )
        {
        }
    };

} // namespace WoRB

#endif // _QUATERNION_H_INCLUDED
