/**
 *  @file      Utilities.cpp
 *  @brief     Implementation of the various GLUT utilites used
 *             by the rigid body test-bed application.
 *  @author    Mikica Kocic
 *  @version   0.5
 *  @date      2012-05-08
 *  @copyright GNU Public License.
 */

#include "Utilities.h"

#include <cstdlib>      // rand
#include <cstdio>       // vsprintf
#include <cstdarg>      // va_list
#include <algorithm>    // min/max

using namespace WoRB;

/////////////////////////////////////////////////////////////////////////////////////////

// Gets a uniform real number in range [0,1)
//
double WoRB::RandomReal () 
{ 
    return double( rand() ) / RAND_MAX;
};

// Gets a quaternion with the given length but with random orientation.
//
Quaternion WoRB::RandomQuaternion( double length )
{
    return Quaternion( RandomReal (), RandomReal (), RandomReal (), RandomReal () )
           .Normalize( length );
}

// Gets a random quaternion uniformly distributed in a 4D box
//
Quaternion WoRB::RandomQuaternion( const Quaternion& min, const Quaternion& max )
{
    return Quaternion(
        min.w + ( max.w - min.w ) * RandomReal (),
        min.x + ( max.x - min.x ) * RandomReal (),
        min.y + ( max.y - min.y ) * RandomReal (),
        min.z + ( max.z - min.z ) * RandomReal ()
    );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Renders the given text to the given location in body-fixed space.
//
void WoRB::RenderText( double x, double y, double z, const char* text )
{
    void* font = GLUT_BITMAP_TIMES_ROMAN_10;

    // Loop through characters displaying them.
    //
    glRasterPos3d( x, y, z );
    for( const char* ch = text; *ch; ++ch ) 
    {
        if ( *ch == '\n' ) {
            // If we meet a newline, then move down by the line-height
            y -= glutBitmapHeight( font ) * 1.2;
            glRasterPos3d( x, y, z );
        }
        glutBitmapCharacter( font, *ch );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Draws the geometry axes, angular velocity and angular momentum
//
void WoRB::RenderStateVariables( const RigidBody& body, const Quaternion& extent )
{
    double maxExtent = std::max( extent.x, std::max( extent.y, extent.z ) ) * 1.2;

    Quaternion pos = body.Position;
    Quaternion w = body.AngularVelocity;
    Quaternion L = body.AngularMomentum;

    glLineWidth( 2 );

    /////////////////////////////////////////////////////////////////////////////////////
    // Display angular velocity

    if ( w.ImSquaredNorm () > 1e-3 ) {
        w.Normalize( maxExtent );
        glColor3d( 0, 0, 0 );
        glBegin( GL_LINES );
        glVertex3d( pos.x, pos.y, pos.z );
        glVertex3d( pos.x + w.x, pos.y + w.y, pos.z + w.z );
        glEnd ();

        RenderText( pos.x + w.x, pos.y + w.y * 1.08, pos.z + w.z, "w" );
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Display angular momentum

    if ( L.ImSquaredNorm () > 1e-3 ) {
        L.Normalize( maxExtent );
        glColor3d( 0.5, 0.5, 0.5 );
        glBegin( GL_LINES );
        glVertex3d( pos.x, pos.y, pos.z );
        glVertex3d( pos.x + L.x, pos.y + L.y, pos.z + L.z );
        glEnd ();

        RenderText( pos.x + L.x, pos.y + L.y * 1.08, pos.z + L.z, "L" );
    }

    glLineWidth( 1 );

    GLTransform bodySpace( body );

    /////////////////////////////////////////////////////////////////////////////////////
    // Display body's X-axis

    glColor3d( 0.8, 0, 0 );
    glBegin( GL_LINES );
    glVertex3d( 0, 0, 0 );
    glVertex3d( extent.x, 0, 0 );
    glEnd ();
    RenderText( extent.x * 1.07, 0, 0, "X" );

    /////////////////////////////////////////////////////////////////////////////////////
    // Display body's Y-axis

    glColor3d( 0, 0.6, 0 );
    glBegin( GL_LINES );
    glVertex3d( 0, 0, 0 );
    glVertex3d( 0, extent.y, 0 );
    glEnd ();
    RenderText( 0, extent.y * 1.07, 0, "Y" );

    /////////////////////////////////////////////////////////////////////////////////////
    // Display body's Z-axis

    glColor3d( 0, 0, 0.8 );
    glBegin( GL_LINES );
    glVertex3d( 0, 0, 0 );
    glVertex3d( 0, 0, extent.z );
    glEnd ();
    RenderText( 0, 0, extent.z * 1.07, "Z" );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Draws the world axes
//
void WoRB::RenderAxes( double length )
{
    glLineWidth( 2 );

    /////////////////////////////////////////////////////////////////////////////////////
    // Display X-axis for the world

    glColor3d( 1, 0, 0 );
    glBegin( GL_LINES );
    glVertex3d( 0, 0, 0 );
    glVertex3d( length, 0, 0 );
    glEnd ();
    RenderText( length + 0.3, 0, 0, "X" );

    /////////////////////////////////////////////////////////////////////////////////////
    // Display Y-axis for the world

    glColor3d( 0, 0.8, 0 );
    glBegin( GL_LINES );
    glVertex3d( 0, 0, 0 );
    glVertex3d( 0, length, 0 );
    glEnd ();
    RenderText( 0, length + 0.3, 0, "Y" );

    /////////////////////////////////////////////////////////////////////////////////////
    // Display Z-axis for the world

    glColor3d( 0, 0, 1 );
    glBegin( GL_LINES );
    glVertex3d( 0, 0, 0 );
    glVertex3d( 0, 0, length );
    glEnd ();
    RenderText( 0, 0, length + 0.3, "Z" );

    glLineWidth( 1 );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Renders the given text to the given location in screen space on the window.
//
int WoRB::RenderPrintf( int x, int y, const char* format, ... )
{
    char buffer[ 2048 ];
    va_list args;
    va_start( args, format );

    #ifdef _WIN32
    #pragma warning(disable:4996)
    #endif
    vsprintf( buffer, format, args );

    va_end( args );

    // Loop through characters displaying them.
    //
    void* font = GLUT_BITMAP_8_BY_13; // GLUT_BITMAP_HELVETICA_10
    glRasterPos2d( x, y );
    for( const char* ch = buffer; *ch; ++ch ) 
    {
        if ( *ch == '\n' ) {
            // Move down by the line-height on new-line
            y -= int( glutBitmapHeight( font ) * 1.2 );
            glRasterPos2d( x, y );
        } 
        else {
            glutBitmapCharacter( font, *ch );
        }
    }

    return y -= int( glutBitmapHeight( font ) * 1.2 );
}
