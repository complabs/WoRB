#ifndef _QTENSOR_H_INCLUDED
#define _QTENSOR_H_INCLUDED

/**
 *  @file      QTensor.h
 *  @brief     Definitions for the QTensor class that represents a quaternionic tensor.
 *  @author    Mikica Kocic
 *  @version   0.5
 *  @date      2012-05-05
 *  @copyright GNU Public License.
 */

#include "Quaternion.h"

namespace WoRB 
{
    /////////////////////////////////////////////////////////////////////////////////////
    /** @class QTensor
     *
     * Encapsulates a quaternionic tensor (q-tensor) as a 4x4 row-major matrix.
     *
     * @note The QTensor class is implemented with inline methods only.
     */
    class QTensor
    {
        const static int Length = 16; //!< Number of components

    public:
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Q-Tensor components                                                  */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Represents tensor components in a *column-major* order.
         *
         * Components are stored contiguously in linear memory as:
         * 
         *   `m.xx`, `m.yx`, `m.zx`... `m.zw`, `m.ww`.
         *
         * Offset of the component is calculated as `row + column * NumRows`
         *
         * @note C/C++ stores matrices in _row-major_ order. MATLAB and OpenGL
         *       stores matrices in column-major order.
         *
         * See <a href="http://en.wikipedia.org/wiki/Row-major_order">
         *     Row-Major Order on Wikipedia </a>
         */
        struct Components
        {
            double  xx, yx, zx, /**/ wx;
            double  xy, yy, zy, /**/ wy;
            double  xz, yz, zz, /**/ wz;
            ///////////////////////////**///////
            double  xw, yw, zw, /**/ ww;
        };

        union
        {
            /** Holds the tensor components as a column-major matrix.
             */
            Components m;

            /** Holds the contigous memory array of the components 
             */
            double data[ Length ];
        };
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Constructors and assignment operators                                */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Represents type of initial matrix data for QTensor constructor.
         */
        enum Initializer
        {
            Uninitialized,  //!< Matrix with uninitialized components
            Normalized,     //!< Matrix with m.ww = 1 and other components set to zero
            Zero,           //!< Matrix with all components set to zero
            Identity        //!< Identity matrix, i.e. 1 on the main diagonal
        };

        /** Creates a tensor with initial contents according to the given type.
         */
        QTensor( Initializer type = Normalized )
        {
            switch( type )
            {
                case Uninitialized:
                    break;

                case Normalized:
                    SetComponents( 0 );
                    m.ww = 1.0;
                    break;

                case Zero:
                    SetComponents( 0 );
                    break;

                case Identity:
                    SetComponents( 0 );
                    m.xx = m.yy = m.zz = m.ww = 1.0;
                    break;
            }
        }

        /** Sets the main diagonal with quaternion components.
         */
        QTensor( const Quaternion& q )
        {
            SetComponents( 0 );
            m.xx = q.x;
            m.yy = q.y;
            m.zz = q.z;
            m.ww = q.w;
        }

        /** Sets the matrix with values along the main diagonal.
         */
        QTensor( double xx, double yy, double zz, double ww = 1.0 )
        {
            SetComponents( 0 );
            m.xx = xx;
            m.yy = yy;
            m.zz = zz;
            m.ww = ww;
        }

        /** Sets the main diagonal with the given value.
         */
        QTensor& operator = ( double valueOnDiagonal )
        {
            SetComponents( 0 );
            m.xx = m.yy = m.zz = m.ww = valueOnDiagonal;
            return *this;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Indexing operators                                                   */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        double& operator [] ( unsigned index )
        {
            return *( &m.xx + index );
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Components setters                                                   */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Sets all components to the same value.
         */
        QTensor& SetComponents( double valueForAllComponents )
        {
            for ( int i = 0; i < Length; ++i ) {
                data[i] = valueForAllComponents;
            }
            return *this;
        }

        /** Sets the q-tensor values from the given three vector components.
         *
         * These are arranged as the three columns of the vector.
         */
        QTensor&  SetColumnVectors(
            const Quaternion& v1, const Quaternion& v2, const Quaternion& v3 )
        {
            m.xx = v1.x ;   m.xy = v2.x ;   m.xz = v3.x ;   m.xw = 0 ;
            m.yx = v1.y ;   m.yy = v2.y ;   m.yz = v3.y ;   m.yw = 0 ;
            m.zx = v1.z ;   m.zy = v2.z ;   m.zz = v3.z ;   m.zw = 0 ;
            m.wx =    0 ;   m.wy =    0 ;   m.wz =    0 ;   m.ww = 1 ;

            return *this;
        }

        /** Sets the matrix to be a skew symmetric matrix based on
         * the given quaternion.
         */
        QTensor& SetSkewSymmetric( const Quaternion& q )
        {
            m.xx =    0 ;   m.xy = -q.z ;   m.xz =  q.y ;   m.xw = 0 ;
            m.yx =  q.z ;   m.yy =    0 ;   m.yz = -q.x ;   m.yw = 0 ;
            m.zx = -q.y ;   m.zy =  q.x ;   m.zz =    0 ;   m.zw = 0 ;
            m.wx =    0 ;   m.wy =    0 ;   m.wz =    0 ;   m.ww = 0 ;

            return *this;
        }

        /** Sets the q-tensor to represent a "left-multiplier quaternion" matrix L(q).
         *
         * See "Rotation of Moment of Inertia Tensor" Mathematica notebook by MBK.
         */
        QTensor& SetLeftMultiplier( const Quaternion& q )
        {
            m.xx =  q.w ;   m.xy = -q.z ;   m.xz =  q.y ;   m.xw = q.x ;
            m.yx =  q.z ;   m.yy =  q.w ;   m.yz = -q.x ;   m.yw = q.y ;
            m.zx = -q.y ;   m.zy =  q.x ;   m.zz =  q.w ;   m.zw = q.z ;
            m.wx = -q.x ;   m.wy = -q.y ;   m.wz = -q.z ;   m.ww = q.w ;

            return *this;
        }

        /** Sets the q-tensor to represent a "right-multiplier quaternion" matrix R(q).
         *
         * See "Rotation of Moment of Inertia Tensor" Mathematica notebook by MBK.
         */
        QTensor& SetRightMultiplier( const Quaternion& q )
        {
            m.xx =  q.w ;   m.xy =  q.z ;   m.xz = -q.y ;   m.xw = q.x ;
            m.yx = -q.z ;   m.yy =  q.w ;   m.yz =  q.x ;   m.yw = q.y ;
            m.zx =  q.y ;   m.zy = -q.x ;   m.zz =  q.w ;   m.zw = q.z ;
            m.wx = -q.x ;   m.wy = -q.y ;   m.wz = -q.z ;   m.ww = q.w ;

            return *this;
        }

        /** Creates a transform matrix from a position and orientation.
         * 
         * This is so called Shoemake's matrix, which is obtained as L(q) * R(q*).
         * See "Rotation of Moment of Inertia Tensor" Mathematica notebook by MBK.
         */
        QTensor&  SetFromOrientationAndPosition(
            const Quaternion& q, const Quaternion& translate )
        {
            m.xx =  1 - 2 * (  q.y * q.y  +  q.z * q.z  ) ;
            m.xy =      2 * (  q.x * q.y  -  q.w * q.z  ) ;
            m.xz =      2 * (  q.x * q.z  +  q.w * q.y  ) ;
            m.xw =  translate.x;

            m.yx =      2 * (  q.x * q.y  +  q.w * q.z  ) ;
            m.yy =  1 - 2 * (  q.x * q.x  +  q.z * q.z  ) ;
            m.yz =      2 * (  q.y * q.z  -  q.w * q.x  ) ;
            m.yw =  translate.y;

            m.zx =      2 * (  q.x * q.z  -  q.w * q.y  ) ;
            m.zy =      2 * (  q.y * q.z  +  q.w * q.x  ) ;
            m.zz =  1 - 2 * (  q.x * q.x  +  q.y * q.y  ) ;
            m.zw =  translate.z;

            m.wx = m.wy = m.wz = 0;  m.ww = 1;

            return *this;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Components getters                                                   */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Gets a quaternion representing one row in the matrix.
         */
        Quaternion Row( unsigned i /*!< Row index */ ) const
        {
            const double* p = data;
            return Quaternion( p[i+12], p[i], p[i+4], p[i+8] );
        }

        /** Gets a quaternion representing one unit base vector (axis).
         * Axis vectors are columns in the matrix.
         * Axis with index 3 corresponds to the position of the transform matrix.
         */
        Quaternion Column( unsigned j /*!< Column index */ ) const
        {
            const double* p = data + j * 4;
            return Quaternion( p[3], p[0], p[1], p[2] );
        }

        /** Gets an OpenGL transform representing a combined translation and rotation.
         */
        void GetGLTransform( double d[Length] ) const
        {
            // We can do simple memcpy since GL also use column-major matrix.
            //
            for ( int i = 0 ; i < Length; ++i ) {
                d[i] = data[i];
            }

            // Otherwise, we should setup GL matrix explicitly:
            // d[0] =  m.xx ;   d[4] =  m.xy ;   d[ 8] =  m.xz ;   d[12] =  m.xw ;
            // d[1] =  m.yx ;   d[5] =  m.yy ;   d[ 9] =  m.yz ;   d[13] =  m.yw ;
            // d[2] =  m.zx ;   d[6] =  m.zy ;   d[10] =  m.zz ;   d[14] =  m.zw ;
            // d[3] =  m.wx ;   d[7] =  m.wy ;   d[11] =  m.wz ;   d[15] =  m.ww ;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Unary operations                                                     */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Flips signs of all the components of the matrix
         */
        QTensor operator - () const
        {
            QTensor result( Uninitialized );

            for ( int i = 0; i < Length; ++i ) {
                result.data[i] = - data[i];
            }

            return result;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Binary operations                                                    */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Does a component-wise addition of this matrix and other matrix.
         */
        QTensor operator + ( const QTensor& T )
        {
            QTensor result( Uninitialized );

            for ( int i = 0; i < Length; ++i ) {
                result.data[i] = data[i] + T.data[i];
            }
            return result;
        }

        /** Does a component-wise addition of this matrix and other matrix.
         */
        QTensor& operator += ( const QTensor& T )
        {
            for ( int i = 0; i < Length; ++i ) {
                data[i] += T.data[i];
            }
            return *this;
        }

        /** Does a component-wise subtraction of this matrix and other matrix.
         */
        QTensor operator - ( const QTensor& T )
        {
            QTensor result( Uninitialized );

            for ( int i = 0; i < Length; ++i ) {
                result.data[i] = data[i] - T.data[i];
            }
            return result;
        }

        /** Does a component-wise subtraction of this matrix and other matrix.
         */
        QTensor& operator -= ( const QTensor& T )
        {
            for ( int i = 0; i < Length; ++i ) {
                data[i] -= T.data[i];
            }
            return *this;
        }

        /** Multiplies this matrix by the given scalar.
         */
        QTensor operator * ( double scalar ) const
        {
            QTensor result( Uninitialized );

            for ( int i = 0; i < Length; ++i ) {
                result.data[i] = data[i] * scalar;
            }
            return result;
        }

        /** Multiplies this matrix by the given scalar.
         */
        QTensor& operator *= ( double scalar )
        {
            for ( int i = 0; i < Length; ++i ) {
                data[i] *= scalar;
            }
            return *this;
        }

        /** Transform the vector part of a quaternion by this matrix.
         */
        Quaternion operator * ( const Quaternion& q ) const
        {
            return Quaternion( 0,
                q.x * m.xx  +  q.y * m.xy  +  q.z * m.xz  +  m.xw,
                q.x * m.yx  +  q.y * m.yy  +  q.z * m.yz  +  m.yw,
                q.x * m.zx  +  q.y * m.zy  +  q.z * m.zz  +  m.zw
            );
        }

        /** Returns a q-tensor which is this matrix multiplied by 
         * the given other q-tensor.
         */
        QTensor operator * ( const QTensor& T ) const
        {
            QTensor result( Uninitialized );

            result.m.xx =  m.xx * T.m.xx  +  m.xy * T.m.yx  +  m.xz * T.m.zx          ;
            result.m.xy =  m.xx * T.m.xy  +  m.xy * T.m.yy  +  m.xz * T.m.zy          ;
            result.m.xz =  m.xx * T.m.xz  +  m.xy * T.m.yz  +  m.xz * T.m.zz          ;
            result.m.xw =  m.xx * T.m.xw  +  m.xy * T.m.yw  +  m.xz * T.m.zw  +  m.xw ;

            result.m.yx =  m.yx * T.m.xx  +  m.yy * T.m.yx  +  m.yz * T.m.zx          ;
            result.m.yy =  m.yx * T.m.xy  +  m.yy * T.m.yy  +  m.yz * T.m.zy          ;
            result.m.yz =  m.yx * T.m.xz  +  m.yy * T.m.yz  +  m.yz * T.m.zz          ;
            result.m.yw =  m.yx * T.m.xw  +  m.yy * T.m.yw  +  m.yz * T.m.zw  +  m.yw ;

            result.m.zx =  m.zx * T.m.xx  +  m.zy * T.m.yx  +  m.zz * T.m.zx          ;
            result.m.zy =  m.zx * T.m.xy  +  m.zy * T.m.yy  +  m.zz * T.m.zy          ;
            result.m.zz =  m.zx * T.m.xz  +  m.zy * T.m.yz  +  m.zz * T.m.zz          ;
            result.m.zw =  m.zx * T.m.xw  +  m.zy * T.m.yw  +  m.zz * T.m.zw  +  m.zw ;

            result.m.wx = result.m.wy = result.m.wz = 0;
            result.m.ww = 1;

            return result;
        }

        /** Multiplies this matrix in place by the given other matrix.
         */
        QTensor& operator *= ( const QTensor& T )
        {
            return *this = *this * T;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Transpose and related methods                                        */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Sets the matrix to be the transpose of the given matrix.
         */
        QTensor& SetTransposeOf( const QTensor& T )
        {
            m.xx = T.m.xx ;   m.xy =  T.m.yx ;   m.xz =  T.m.zx ;   m.xw =  T.m.wx ;
            m.yx = T.m.xy ;   m.yy =  T.m.yy ;   m.yz =  T.m.zy ;   m.yw =  T.m.wy ;
            m.zx = T.m.xz ;   m.zy =  T.m.yz ;   m.zz =  T.m.zz ;   m.zw =  T.m.wz ;
            m.wx = T.m.xw ;   m.wy =  T.m.yw ;   m.wz =  T.m.zw ;   m.ww =  T.m.ww ;

            return *this;
        }

        /** Returns a new matrix containing the transpose of this matrix.
         */
        QTensor Transpose () const
        {
            return QTensor( Uninitialized ).SetTransposeOf( *this );
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Determinant, matrix Inverse and related methods                      */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Returns the determinant of the matrix.
         */
        double Determinant () const
        {
            return - m.zx * m.yy * m.xz
                   + m.yx * m.zy * m.xz
                   + m.zx * m.xy * m.yz
                   - m.xx * m.zy * m.yz
                   - m.yx * m.xy * m.zz
                   + m.xx * m.yy * m.zz;
        }

        /** Sets the matrix to be the inverse of the given matrix.
         */
        QTensor& SetInverseOf( const QTensor& T )
        {
            double det_T = T.Determinant();

            // Make sure the determinant is non-zero.
            //
            if ( det_T == 0 ) {
                SetComponents( 0 );
                return *this;
            }

            m.xx =  - T.m.zy * T.m.yz  +  T.m.yy * T.m.zz;
            m.yx =    T.m.zx * T.m.yz  -  T.m.yx * T.m.zz;
            m.zx =  - T.m.zx * T.m.yy  +  T.m.yx * T.m.zy;

            m.xy =    T.m.zy * T.m.xz  -  T.m.xy * T.m.zz;
            m.yy =  - T.m.zx * T.m.xz  +  T.m.xx * T.m.zz;
            m.zy =    T.m.zx * T.m.xy  -  T.m.xx * T.m.zy;

            m.xz =  - T.m.yy * T.m.xz  +  T.m.xy * T.m.yz;
            m.yz =    T.m.yx * T.m.xz  -  T.m.xx * T.m.yz;
            m.zz =  - T.m.yx * T.m.xy  +  T.m.xx * T.m.yy;

            m.xw =    T.m.zy * T.m.yz * T.m.xw
                    - T.m.yy * T.m.zz * T.m.xw
                    - T.m.zy * T.m.xz * T.m.yw
                    + T.m.xy * T.m.zz * T.m.yw
                    + T.m.yy * T.m.xz * T.m.zw
                    - T.m.xy * T.m.yz * T.m.zw;

            m.yw =  - T.m.zx * T.m.yz * T.m.xw
                    + T.m.yx * T.m.zz * T.m.xw
                    + T.m.zx * T.m.xz * T.m.yw
                    - T.m.xx * T.m.zz * T.m.yw
                    - T.m.yx * T.m.xz * T.m.zw
                    + T.m.xx * T.m.yz * T.m.zw;

            m.zw =   T.m.zx * T.m.yy * T.m.xw
                   - T.m.yx * T.m.zy * T.m.xw
                   - T.m.zx * T.m.xy * T.m.yw
                   + T.m.xx * T.m.zy * T.m.yw
                   + T.m.yx * T.m.xy * T.m.zw
                   - T.m.xx * T.m.yy * T.m.zw;

            for ( int i = 0; i < Length; ++ i ) {
                data[i] /= det_T;
            }
            return *this;
        }

        /** Returns a new matrix containing the inverse of this matrix.
         */
        QTensor Inverse () const
        {
            return QTensor( Uninitialized ).SetInverseOf( *this );
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Spatial transformations related methods                              */
        /////////////////////////////////////////////////////////////////////////////////
                                                                                   /*@{*/
        /** Transform the vector part of a quaternion by this matrix.
         */
        Quaternion operator () ( const Quaternion& q ) const
        {
            return Quaternion( 0,
                q.x * m.xx  +  q.y * m.xy  +  q.z * m.xz  +  m.xw,
                q.x * m.yx  +  q.y * m.yy  +  q.z * m.yz  +  m.yw,
                q.x * m.zx  +  q.y * m.zy  +  q.z * m.zz  +  m.zw
            );
        }

        /** Transform the given quaternion by the inverse of this matrix.
         */
        Quaternion TransformInverse( const Quaternion &q ) const
        {
            Quaternion del = q - Quaternion( 0, m.xw, m.yw, m.zw );

            return Quaternion( 0,
                del.x * m.xx  +  del.y * m.yx  +  del.z * m.zx,
                del.x * m.xy  +  del.y * m.yy  +  del.z * m.zy,
                del.x * m.xz  +  del.y * m.yz  +  del.z * m.zz
            );
        }

        /** Transforms a tensor from one to another frame of reference.
         *
         *  Equivalent to: `result = (*this) * T * Transpose()`
         */
        QTensor operator () ( const QTensor& T ) const
        {
            QTensor result( Uninitialized );

            double t_xx =  m.xx * T.m.xx  +  m.xy * T.m.yx  +  m.xz * T.m.zx ;
            double t_xy =  m.xx * T.m.xy  +  m.xy * T.m.yy  +  m.xz * T.m.zy ;
            double t_xz =  m.xx * T.m.xz  +  m.xy * T.m.yz  +  m.xz * T.m.zz ;

            double t_yx =  m.yx * T.m.xx  +  m.yy * T.m.yx  +  m.yz * T.m.zx ;
            double t_yy =  m.yx * T.m.xy  +  m.yy * T.m.yy  +  m.yz * T.m.zy ;
            double t_yz =  m.yx * T.m.xz  +  m.yy * T.m.yz  +  m.yz * T.m.zz ;

            double t_zx =  m.zx * T.m.xx  +  m.zy * T.m.yx  +  m.zz * T.m.zx ;
            double t_zy =  m.zx * T.m.xy  +  m.zy * T.m.yy  +  m.zz * T.m.zy ;
            double t_zz =  m.zx * T.m.xz  +  m.zy * T.m.yz  +  m.zz * T.m.zz ;

            result.m.xx =  t_xx * m.xx    +  t_xy * m.xy    +  t_xz * m.xz   ;
            result.m.xy =  t_xx * m.yx    +  t_xy * m.yy    +  t_xz * m.yz   ;
            result.m.xz =  t_xx * m.zx    +  t_xy * m.zy    +  t_xz * m.zz   ;
            result.m.xw =  0                                                 ;

            result.m.yx =  t_yx * m.xx    +  t_yy * m.xy    +  t_yz * m.xz   ;
            result.m.yy =  t_yx * m.yx    +  t_yy * m.yy    +  t_yz * m.yz   ;
            result.m.yz =  t_yx * m.zx    +  t_yy * m.zy    +  t_yz * m.zz   ;
            result.m.yw =  0                                                 ;

            result.m.zx =  t_zx * m.xx    +  t_zy * m.xy    +  t_zz * m.xz   ;
            result.m.zy =  t_zx * m.yx    +  t_zy * m.yy    +  t_zz * m.yz   ;
            result.m.zz =  t_zx * m.zx    +  t_zy * m.zy    +  t_zz * m.zz   ;
            result.m.zw =  0                                                 ;

            result.m.wx = result.m.wy = result.m.ww = 0;
            result.m.ww = 1;

            return result;
        }

        /** Transform the given matrix by the inverse of this matrix.
         *
         *  Equivalent to: `result = Transpose() * T * (*this)`
         */
        QTensor TransformInverse( const QTensor &T ) const
        {
            QTensor result( Uninitialized );

            double t_xx =  m.xx * T.m.xx  +  m.yx * T.m.yx  +  m.zx * T.m.zx ;
            double t_xy =  m.xx * T.m.xy  +  m.yx * T.m.yy  +  m.zx * T.m.zy ;
            double t_xz =  m.xx * T.m.xz  +  m.yx * T.m.yz  +  m.zx * T.m.zz ;

            double t_yx =  m.xy * T.m.xx  +  m.yy * T.m.yx  +  m.zy * T.m.zx ;
            double t_yy =  m.xy * T.m.xy  +  m.yy * T.m.yy  +  m.zy * T.m.zy ;
            double t_yz =  m.xy * T.m.xz  +  m.yy * T.m.yz  +  m.zy * T.m.zz ;

            double t_zx =  m.xz * T.m.xx  +  m.yz * T.m.yx  +  m.zz * T.m.zx ;
            double t_zy =  m.xz * T.m.xy  +  m.yz * T.m.yy  +  m.zz * T.m.zy ;
            double t_zz =  m.xz * T.m.xz  +  m.yz * T.m.yz  +  m.zz * T.m.zz ;

            result.m.xx =  t_xx * m.xx    +  t_xy * m.yx    +  t_xz * m.zx   ;
            result.m.xy =  t_xx * m.xy    +  t_xy * m.yy    +  t_xz * m.zy   ;
            result.m.xz =  t_xx * m.xz    +  t_xy * m.yz    +  t_xz * m.zz   ;
            result.m.xw =  0                                                 ;

            result.m.yx =  t_yx * m.xx    +  t_yy * m.yx    +  t_yz * m.zx   ;
            result.m.yy =  t_yx * m.xy    +  t_yy * m.yy    +  t_yz * m.zy   ;
            result.m.yz =  t_yx * m.xz    +  t_yy * m.yz    +  t_yz * m.zz   ;
            result.m.yw =  0                                                 ;

            result.m.zx =  t_zx * m.xx    +  t_zy * m.yx    +  t_zz * m.zx   ;
            result.m.zy =  t_zx * m.xy    +  t_zy * m.yy    +  t_zz * m.zy   ;
            result.m.zz =  t_zx * m.xz    +  t_zy * m.yz    +  t_zz * m.zz   ;
            result.m.zw =  0                                                 ;

            result.m.wx = result.m.wy = result.m.ww = 0;
            result.m.ww = 1;

            return result;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
    };

} // namespace WoRB

#endif // _QTENSOR_H_INCLUDED
