#ifndef _MEX_WORB_H_INCLUDED
#define _MEX_WORB_H_INCLUDED

/**
 *  @file      mexWoRB.h
 *  @brief     Definitions for `Matrix`, `Scalar`, `Logical` and `String` classes
 *             which simplify access to MATLAB array objects.
 *  @author    Mikica Kocic
 *  @version   0.3
 *  @date      2012-05-14
 *  @copyright GNU Public License.
 */

#include "mex.h"

namespace Mex
{
    /** Represents a simple matrix class that keeps components in a column-major order.
     */
    class Matrix
    {
        /** Holds the number of rows in matrix.
         */
        mwIndex NRows;

        /** Holds the number of columns in matrix.
         */
        mwIndex NCols;

        /** Holds the matrix data in column-major order.
         */
        double* Data;

        /** Initializes the matrix to refer to the given MATLAB array.
         * This method *does not* copy data from the given MATLAB array.
         */
        void Initialize( const mxArray* arg, const char* argDesc )
        {
            if ( arg == NULL ) {
                return;
            }

            // We expect a scalar, vector or a matrix of real numbers.
            //
            int ndim = mxGetNumberOfDimensions( arg );
            if ( ! mxIsDouble( arg ) || mxIsComplex( arg ) || ndim > 2 )
            {
                WoRB::SevereError( "WoRB:invarg",
                    "'%s' must be a scalar, vector or a matrix of real numbers.", 
                    argDesc
                );
            }

            // Get the matrix dimensions and the first element of the real data
            const mwSize* dims = mxGetDimensions( arg );
            NRows = ndim >= 1 ? dims[0] : 1;
            NCols = ndim >= 2 ? dims[1] : 1;
            Data  = mxGetPr( arg );
        }
    
    public:
    
        /** Creates an uninitialized matrix allocated using `mxAlloc`.
         */
        Matrix( mwIndex rowCount, mwIndex columnCount )
            : NRows( rowCount )
            , NCols( columnCount )
            , Data( NULL )
        {
            Data = (double*)mxMalloc( MemorySize () ); 
        }

        /** Constructs the matrix from a `mexFunction` argument given by the argument number.
         */
        Matrix( int nArgIn, const mxArray* argIn[], int argNo, const char* argDesc )
            : NRows( 0 )
            , NCols( 0 )
            , Data( NULL )
        {
            if ( argNo < nArgIn ) {
                Initialize( argIn[ argNo ], argDesc );
            }
        }

        /** Constructs the matrix from MATLAB variable from the specified workspace.
         */
        Matrix( const char* varName, const char* workspace = "caller" )
        {
            Initialize( mexGetVariable( workspace, varName ), varName );
        }

        /** Constructs the matrix from the given MATLAB structure field.
         */
        Matrix( const mxArray* obj, mwIndex index, const char* fieldName )
        {
            Initialize( mxGetField( obj, index, fieldName ), fieldName );
        }

        /** Destructor. Does nothing.
         * MATLAB automatically frees all data allocated by `mxMalloc`.
         */
        ~Matrix ()
        {
        }

        /** Converts an instance to a MATLAB double matrix (allocated on MATLAB heap)
         */
        operator mxArray* ()
        {
            // Note: The array creation with mxCreateDoubleMatrix initializes
            // the array memory by filling it with zeros. The following code avoids
            // this unnecessary initialization.

            // Create an uninitialized empty array
            mxArray* result = mxCreateDoubleMatrix( 0, 0, mxREAL );
        
            // Set dimensions (note that either of NRows/NCols may be 0)
            mxSetM( result, NRows );  // Set M = row count
            mxSetN( result, NCols );  // Set N = column count

            if ( NRows && NCols ) {
                mxSetData( result, Data ); // Set mxArray data to point to output
            }

            return result;
        }

        /** Returns true if the instance is an empty array.
         */
        bool IsEmpty () const
        {
            return NRows * NCols == 0;
        }

        /** Returns true if the instance is a scalar (1x1 matrix).
         */
        bool IsScalar () const
        {
            return NRows == 1 && NCols == 1;
        }

        /** Returns true if the instance is a vector (1xM or Nx1 matrix) or a scalar
         */
        bool IsVector () const
        {
            return ( NRows == 1 && NCols > 0 ) || ( NCols == 1 && NRows > 0 );
        }

        /** Returns true if the instance is a row vector (1xM matrix).
         */
        bool IsRowVector () const
        {
            return NRows == 1 && NCols > 0;
        }

        /** Returns true if the instance is a column vector (Nx1 matrix).
         */
        bool IsColumnVector () const
        {
            return NCols == 1 && NRows > 0;
        }

        /** Returns true if the instance is non-empty matrix that is not a vector.
         */
        bool IsFullMatrix () const
        {
            return NCols > 1 && NRows > 1;
        }

        /** Checks the sizes of each dimension of the matrix.
         */
        bool IsSize( mwIndex M, mwIndex N ) const
        {
            return M == NRows && N == NCols;
        }

        /** Returns true if the specified index points to a valid row.
         */
        bool IsValidRow( mwIndex r ) const
        {
            return 0 <= r && r < NRows;
        }

        /** Returns true if the specified index points to a valid column.
         */
        bool IsValidColumn( mwIndex c ) const
        {
            return 0 <= c && c < NCols;
        }

        /** Returns the length of the (linear) array of the matrix.
         */
        mwIndex Length () const
        {
            return NRows * NCols;
        }
    
        /** Gets the number of rows in the matrix.
         */
        mwIndex GetM () const
        {
            return NRows;
        }

        /** Gets the number of columns in the matrix.
         */
        mwIndex GetN () const
        {
            return NCols;
        }

        /** Gets the memory size (in bytes) occupied by array.
         */
        mwSize MemorySize () const
        {
            return NRows * NCols * sizeof( double );
        }
    
        /** Verifies sizes of each matrix dimension.
         * Reports severe error if dimensions do not agree.
         */
        void VerifyDims( mwIndex M, mwIndex N, const char* error_message ) const
        {
            if ( IsSize( M, N ) || ( Length () == 0 && M * N == 0 ) ) {
                return; // OK
            }

            WoRB::SevereError( 
                "Matrix:VerifyDims:invsize",
                "%s; Required dimensions [ %d × %d ] have only [ %d × %d ]", 
                error_message, M, N, NRows, NCols
            ); 
        }

        //////////////////////////////////////////////////////////////////////////////////////

        /** Returns matrix element given by the linear index.
         */
        double& operator ()( mwIndex ix )
        {
            if ( ix < 0 || ix >= Length () ) {
                WoRB::SevereError( 
                    "Matrix:elem:invdim",
                    "Linear index %d outside dimensions [ %d × %d ]", ix, NRows, NCols
                ); 
            }

            return Data[ ix ];
        }

        /** Returns matrix element given by the linear index.
         */
        const double& operator ()( mwIndex ix ) const
        {
            if ( ix < 0 || ix >= Length () ) {
                WoRB::SevereError( 
                    "Matrix:elem:invdim",
                    "Linear index %d outside dimensions [ %d × %d ]", ix, NRows, NCols
                ); 
            }

            return Data[ ix ];
        }

        /** Returns matrix element given by row and column index.
         */
        double& operator ()( mwIndex r, mwIndex c )
        {
            mwIndex ix = r + c * NRows;
            if ( ix < 0 || ix >= Length () ) {
                WoRB::SevereError( 
                    "Matrix:elem:invdim",
                    "Index (%d,%d) outside dimensions [ %d × %d ]", r, c, NRows, NCols
                ); 
            }

            return Data[ ix ];
        }

        /** Returns matrix element given by row and column index.
         */
        const double& operator ()( mwIndex r, mwIndex c ) const
        {
            mwIndex ix = r + c * NRows;
            if ( ix < 0 || ix >= Length () ) {
                WoRB::SevereError( 
                    "Matrix:elem:invdim",
                    "Index (%d,%d) outside dimensions [ %d × %d ]", r, c, NRows, NCols
                ); 
            }

            return Data[ ix ];
        }
    };

    /** Represents a MATLAB scalar double value.
     */
    class Scalar
    {
        double value;

        /** Initializes the instance from a MATLAB array of doubles.
         */
        void Initialize( const mxArray* arg, const char* argDesc, bool defaultValue = false )
        {
            if ( arg == NULL ) {
                value = defaultValue;
                return;
            }

            // Parse value from different argument types
            // int ndim = mxGetNumberOfDimensions( arg );
            mwSize len = mxGetNumberOfElements( arg );

            if ( mxIsDouble( arg ) && ! mxIsComplex( arg ) && len >= 1 )
            {
                double* data = mxGetPr( arg );
                value = data != NULL ? data[0] : 0.0;
                return;
            }

            // If we couldn't parse real value, return an error
            WoRB::SevereError( "WoRB:parseDouble:invarg",
                "'%s' must be a real number.", 
                argDesc
            );
        }

    public:

        /** Constructs the instance from the given MATLAB array of doubles.
         */
        Scalar( const mxArray* arg, const char* argDesc )
        {
            Initialize( arg, argDesc );
        }
    
        /** Constructs the instance from the given MATLAB structure field.
         */
        Scalar( const mxArray* obj, mwIndex index, const char* fieldName )
        {
            Initialize( mxGetField( obj, index, fieldName ), fieldName );
        }

        operator double () const
        {
            return value;
        }

        operator int () const
        {
            return int( value );
        }

        operator unsigned () const
        {
            return unsigned( value );
        }

        operator double& ()
        {
            return value;
        }

        /** Converts the instance to MATLAB real scalar.
         */
        mxArray* toMxArray()
        {
            return mxCreateDoubleScalar( value );
        }
    };

    /** Represents a MATLAB logical value.
     */
    class Logical
    {
        bool value;

        /** Initializes the instance from a MATLAB array of doubles.
         */
        void Initialize( const mxArray* arg, const char* argDesc, bool defaultValue )
        {
            if ( arg == NULL ) {
                value = defaultValue;
                return;
            }

            // Parse value from different argument types
            // int ndim = mxGetNumberOfDimensions( arg );
            mwSize len = mxGetNumberOfElements( arg );

            if ( mxIsLogicalScalar( arg ) && len >= 1 ) 
            {
                value = mxGetLogicals( arg )[0];
                return;
            }
            else if ( mxIsDouble( arg ) && ! mxIsComplex( arg ) && len >= 1 )
            {
                double* data = mxGetPr( arg );
                value = data != NULL && data[0] != 0;
                return;
            }

            // If we couldn't parse logical value, return an error
            WoRB::SevereError( "WoRB:parseLogical:invarg",
                "'%s' must be a logical value or a real number.", 
                argDesc
            );
        }

    public:

        /** Constructs the instance from the given MATLAB array of doubles.
         */
        Logical( const mxArray* arg, const char* argDesc, bool defaultValue = false )
        {
            Initialize( arg, argDesc, defaultValue );
        }
    
        /** Constructs the instance from the given MATLAB structure field.
         */
        Logical( const mxArray* obj, mwIndex index, const char* fieldName,
            bool defaultValue = false )
        {
            Initialize( mxGetField( obj, index, fieldName ), fieldName, defaultValue );
        }

        bool operator ! () const
        {
            return !value;
        }

        operator bool () const
        {
            return value;
        }

        /** Converts the instance to MATLAB logical scalar.
         */
        mxArray* toMxArray()
        {
            return mxCreateLogicalScalar( value );
        }

        /** Sets the new value.
         */
        Logical& operator = ( bool v )
        {
            value = v;
            return* this;
        }
    };

    /** Represents a MATLAB character string value.
     */
    class String
    {
        char* value;

        /** Initializes the instance from a MATLAB `mxArray`.
         */
        void Initialize( const mxArray* arg, const char* argDesc )
        {
            if ( arg == NULL || ! mxIsChar( arg ) ) {
                // If we couldn't parse character string, return an error
                WoRB::SevereError( "WoRB:parseString:invarg",
                    "'%s' must be a character array.", 
                    argDesc
                );
            }

            mwSize len = mxGetNumberOfElements( arg );
            value = (char*)mxMalloc( len + 1 );
            mxGetString( arg, value, len + 1 );
        }
    
    public:

        /** Constructs the instance from the given MATLAB character array.
         */
        String( const mxArray* arg, const char* argDesc )
        {
            Initialize( arg, argDesc );
        }
    
        /** Constructs the instance from the given MATLAB structure field.
         */
        String( const mxArray* obj, mwIndex index, const char* fieldName )
        {
            Initialize( mxGetField( obj, index, fieldName ), fieldName );
        }
    
        /** Destructor. Does nothing.
         * MATLAB automatically frees all data allocated by `mxMalloc`.
         */
        ~String ()
        {
        }

        /** Gets pointer to internal character string.
         */
        operator const char* () const
        {
            return value;
        }

        /** Converts the instance to MATLAB string.
         */
        mxArray* toMxArray()
        {
            return mxCreateString( value );
        }

        /** Returns true if this instance has the same contents as the given string.
         */
        bool operator == ( const char* v )
        {
            for ( const char* p = value; *p && *v; ++p, ++v ) {
                if ( *p != *v ) {
                    return false;
                }
            }
            return true;
        }

        /** Returns true if this instance does not have the same contents as the given string.
         */
        bool operator != ( const char* v )
        {
            return ! operator ==( v );
        }
    };

} // namespace Mex

#endif // _MEX_WORB_H_INCLUDED
