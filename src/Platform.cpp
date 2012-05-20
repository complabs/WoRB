/**
 *  @file      Platform.cpp
 *  @brief     Implementation of platform dependent functions (like Pause()).
 *  @author    Mikica Kocic
 *  @version   0.2
 *  @date      2012-04-29
 *  @copyright GNU Public License.
 */

#ifdef MATLAB_MEX_FILE
    #include "mex.h"
#endif

#ifdef _WIN32
    #include <Windows.h>
    #pragma warning(disable:4996) // vsprintf warning
#else
    #include <unistd.h>
#endif

#include <cstdarg>    // va_list
#include <cstdio>     // vsprintf
#include <cstdlib>    // exit

namespace WoRB
{
    void Pause( unsigned long ms )
    {
        #ifdef _WIN32
            Sleep( ms );
        #else
            usleep( ms * 1000ul );
        #endif
    }

    void Printf( const char* format, ... )
    {
        va_list args;
        va_start( args, format );

        char buffer[ 2048 ];
        vsprintf( buffer, format, args );

        va_end( args );

        #ifdef MATLAB_MEX_FILE
            mexPrintf( "%s", buffer );
            // mexEvalString( "drawnow;" ); // = flush
        #else
            fputs( buffer, stdout );
            fflush( stdout );
        #endif
    }

    void glutForegroundWindow ()
    {
        #ifdef _WIN32

        HWND hWnd = FindWindow( TEXT("FREEGLUT"), NULL );
        if ( hWnd ) {
            SetForegroundWindow( hWnd );
        }

        #endif
    }

    // Handles glut errors.
    //
    void OnGlutError( const char* format, va_list args )
    {
        char buffer[ 2048 ];
        vsprintf( buffer, format, args );

        Printf( "WoRB: %s\n", buffer );
    }

    // Handles glut warnings.
    //
    void OnGlutWarning( const char* format, va_list args )
    {
        char buffer[ 2048 ];
        vsprintf( buffer, format, args );

        Printf( "WoRB: %s\n", buffer );
    }

    // Reports a severe error and quits.
    //
    void SevereError( const char* errorId, const char* errorMsg, ... )
    {
        va_list args;
        va_start( args, errorMsg );
        char buffer[ 1024 ];
        vsprintf( buffer, errorMsg, args );
        va_end( args );

        #if MATLAB_MEX_FILE
            mexErrMsgIdAndTxt( errorId, "%s", buffer );
        #else
            fputs( errorId, stderr );
            fputs( "\n", stderr );
            fputs( buffer, stderr );
            fflush( stderr );
            exit(0);
        #endif
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Redefinitions of new/delete C++ operators to use MATLAB's mxMalloc/mxFree.

#ifdef MATLAB_MEX_FILE

    #include <new>

    void* operator new ( size_t size);
    void* operator new ( size_t size, const std::nothrow_t& ) throw();
    void* operator new[] ( size_t size );
    void* operator new[] ( size_t size, const std::nothrow_t& ) throw();
    void operator delete ( void* p );
    void operator delete[] ( void* p );

    void* operator new( size_t size )
    {
	    void* ptr = mxMalloc( size );
	    if ( ! ptr ) {
		    throw std::bad_alloc ();
	    }
        mexMakeMemoryPersistent( ptr );

        #ifdef DEBUG
        WoRB::Printf( "WoRB: Alloc %p, new %u octets\n", ptr, size );
        #endif

        return ptr;
    }

    void* operator new( size_t size, const std::nothrow_t& ) throw()
    {
	    void* ptr = mxMalloc( size );
        mexMakeMemoryPersistent( ptr );
    
        #ifdef DEBUG
        WoRB::Printf( "WoRB: Alloc %p, new %u octets (nothrow)\n", ptr, size );
        #endif

	    return ptr;
    }

    void* operator new[] ( size_t size )
    {
	    void* ptr = mxMalloc( size );
	    if ( ! ptr ) {
		    throw std::bad_alloc ();
	    }

        mexMakeMemoryPersistent( ptr );

        #ifdef DEBUG
        WoRB::Printf( "WoRB: Alloc %p, new[] %u octets\n", ptr, size );
        #endif

        return ptr;
    }

    void* operator new[] ( size_t size, const std::nothrow_t& ) throw()
    {
	    void* ptr = mxMalloc( size );

        #ifdef DEBUG
        WoRB::Printf( "WoRB: Alloc %p, new[] %u octets (nothrow)\n", ptr, size );
        #endif

	    return ptr;
    }

    void operator delete( void* ptr )
    {
	    if ( ptr )
        {
            #ifdef DEBUG
            WoRB::Printf( "WoRB: Free %p, delete\n", ptr );
            #endif

		    mxFree( ptr );
	    }
    }
    void operator delete[] ( void* ptr )
    {
	    if ( ptr )
        {
            #ifdef DEBUG
            WoRB::Printf( "WoRB: Free %p, delete[]\n", ptr );
            #endif

		    mxFree( ptr );
	    }
    }

#endif // MATLAB_MEX_FILE
