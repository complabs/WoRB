#ifndef _WORB_UTILITIES_H_INCLUDED
#define _WORB_UTILITIES_H_INCLUDED

/**
 *  @file      Utilities.h
 *  @brief     Declarations for various GLUT utilites for a rigid body application.
 *  @author    Mikica B Kocic
 *  @version   0.2
 *  @date      2012-05-10
 *  @copyright GNU Public License.
 */

#include "WoRB.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Include Freeglut

/** Disable GLUT ataxit workaround on windows.
 *
 *  Win32 has an annoying issue where there are multiple C run-time
 *  libraries (CRTs). If the executable is linked with a different CRT
 *  from the GLUT DLL, the GLUT DLL will not share the same CRT static
 *  data seen by the executable.  In particular, atexit callbacks registered
 *  in the executable will not be called if GLUT calls its (different)
 *  exit routine).  GLUT is typically built with the
 *  "/MD" option (the CRT with multithreading DLL support), but the Visual
 *  C++ linker default is "/ML" (the single threaded CRT).
 *
 *  One workaround to this issue is requiring users to always link with
 *  the same CRT as GLUT is compiled with.  That requires users supply a
 *  non-standard option.  GLUT 3.7 has its own built-in workaround where
 *  the executable's "exit" function pointer is covertly passed to GLUT.
 *  GLUT then calls the executable's exit function pointer to ensure that
 *  any "atexit" calls registered by the application are called if GLUT
 *  needs to exit.
 *
 *  To avoid the atexit workaround, define GLUT_DISABLE_ATEXIT_HACK.
 */
#define GLUT_DISABLE_ATEXIT_HACK

#include <GL/freeglut.h>

/////////////////////////////////////////////////////////////////////////////////////////

namespace WoRB
{
    /////////////////////////////////////////////////////////////////////////////////////
    // Global Utilities functions

    /** Gets a uniform real number in range [0,1).
     */
    extern double RandomReal ();

    /** Gets a quaternion of the given length having a random orientation.
     */
    extern Quaternion RandomQuaternion( double length = 1.0 );

    /** Gets a random quaternion uniformly distributed in a 4D box
     */
    extern Quaternion RandomQuaternion( const Quaternion& min, const Quaternion& max );

    /** Renders the given text to the given location in body-fixed space.
     */
    extern void RenderText( double x, double y, double z, const char* text );

    /** Renders the given text to the given location in screen space on the window.
     * @return The next y location bellow the text.
     */
    extern int RenderPrintf( int x, int y, const char* format, ... );

    /** Draws the geometry axes, angular velocity and angular momentum
     */
    extern void RenderStateVariables( const RigidBody& body, const Quaternion& extent );

    /** Draws the world axes
     */
    extern void RenderAxes( double length );

    /** Pauses execution for the given amount of milliseconds.
     */
    extern void Pause( unsigned long ms );

    /** Brings the current GLUT window to the foreground.
     */
    extern void glutForegroundWindow ();

    /////////////////////////////////////////////////////////////////////////////////////
    // FreeGLUT v2.8.0 specifics

    #ifdef GLUT_HAS_MULTI   // Appears first time in freeglut v2.8.0

    /** Handles glut errors.
     */
    extern void OnGlutError( const char* format, va_list args );

    /** Handles glut warnings.
      */
    extern void OnGlutWarning( const char* format, va_list args );

    #endif

    /////////////////////////////////////////////////////////////////////////////////////

    /** Interface for a GLUT-rendered rigid body.
     */
    class GLUT_Renderer
    {
    public:
        /////////////////////////////////////////////////////////////////////////////////

        enum RenderType
        {
            BodyShape,   //!< Render the body itself
            BodyAxes,    //!< Render the body axes
            BodyShadow,  //!< Render the body shadow (flattened body)
            FloorMirror  //!< Render floor mirror image of the body
        };

        /////////////////////////////////////////////////////////////////////////////////

        /** Represents a color defined red, green, blue and alpha channels.
         */
        struct Colorf
        {
            float R;  //!< Holds red color component.
            float G;  //!< Holds green color component.
            float B;  //!< Holds blue color component.
            float A;  //!< Holds aplpha (transparency) channel.

            /** Default constructor; black opaque color.
             */
            Colorf ()
                : R( 0.0f ), G( 0.0f ), B( 0.0f ), A( 1.0f )
            {
            }

            /** Constructs color from the given components.
             */
            Colorf( float red, float green, float blue, float alpha = 1.0f )
                : R( red ), G( green ), B( blue ), A( alpha )
            {
            }

            /** Conctructs color from the elements of quaternion.
             */
            Colorf( const Quaternion& q )
                : R( float(q.x) ), G( float(q.y) ), B( float(q.z) ), A( float(q.w) )
            {
            }
        };

        /////////////////////////////////////////////////////////////////////////////////

        /** Indicates whether to track objects trajectory.
         */
        bool ShowTrajectory;

        /** Holds the shape color, used when body is active (moving).
         */
        Colorf ActiveColor;

        /** Holds the shape color, used when body is inactive (not-moving).
         */
        Colorf InactiveColor;

        /////////////////////////////////////////////////////////////////////////////////

        /** Default construtor; enables tracking objects trajectory by default.
         */
        GLUT_Renderer ()
            : ShowTrajectory( true )
        {
        }

        /** Gets underlying rigid body.
         */
        virtual RigidBody& GetBody () = 0;

        /** Gets underlying geometry.
         */
        virtual Geometry& GetGeometry () = 0;

        /** Draws the required geometry (specified by RenderType) of the rigid body.
         */
        virtual void Render( RenderType type ) = 0;

        /** Draws the wireframe of the rigid body.
         */
        virtual void RenderWireframe( double* transform ) = 0;

        /** Virtual destructor (must have; even if not needed).
         */
        virtual ~GLUT_Renderer () {}
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Establishes temporarily a GL transform from body-fixed into world frame.
     *
     * Usage: usually in a statement-block, like: <code>

       { 
          GLTransform _( body );  

          // ...code-here... 
       }

       </code>
     */
    class GLTransform
    {
    public:
        /** Constructor; saves the current and establishes a new transformation.
         */
        GLTransform( const RigidBody& body, bool flattenHeight = false )
        {
            // Get the OpenGL transformation for the rigid body
            //
            GLdouble mat[ 16 ];
            body.ToWorld.GetGLTransform( mat );

            glPushMatrix (); // Save the current transformation

            if ( flattenHeight ) {
                glScaled( 1.0, 0, 1.0 );
            }

            glMultMatrixd( mat ); // Transform from body-fixed into world frame
        }

        /** Constructor; saves the current and establishes a new transformation.
         */
        GLTransform( const double* matrix )
        {
            glPushMatrix (); // Save the current transformation
            glMultMatrixd( matrix ); // Transform from body-fixed into world frame
        }

        /** Destructor; restores the earlier transformation.
         */
        ~GLTransform ()
        {
            glPopMatrix ();
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Establishes temporarily orthogonal projection in screen coordinates.
     */
    class GLOrthoScreen
    {
    public:
        /** Constructor; saves the current and establishes a new transformation.
         */
        GLOrthoScreen ()
        {
            glDisable( GL_DEPTH_TEST );

            // Temporarily set up the view in orthographic projection.
            //
            glMatrixMode( GL_PROJECTION );

            glPushMatrix ();
            glLoadIdentity ();
            glOrtho( /* left   */  0.0, /* right */ glutGet( GLUT_WINDOW_WIDTH ), 
                     /* bottom */  0.0, /* top   */ glutGet( GLUT_WINDOW_HEIGHT ), 
                     /* zNear  */ -1.0, /* zFar  */ 1.0
            );

            glMatrixMode( GL_MODELVIEW );

            glPushMatrix ();
            glLoadIdentity ();
        }

        /** Destructor; restores the earlier transformation.
         */
        ~GLOrthoScreen ()
        {
            // Go back to projection mode
            //
            glPopMatrix ();
            glMatrixMode( GL_PROJECTION );

            glPopMatrix ();
            glMatrixMode( GL_MODELVIEW );

            glEnable( GL_DEPTH_TEST );
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Encapsulates a rigid body with geometry of a sphere.
     */
    class Ball : public Sphere, public RigidBody, public GLUT_Renderer
    {
        const static int slices = 20; //!< Number of slices for glutSolidSphere()
        const static int stacks = 20; //!< Number of stacks for glutSolidSphere()

    public:

        /////////////////////////////////////////////////////////////////////////////////

        /** Creates a ball at the given location.
         */
        Ball( 
            Quaternion position, Quaternion orientation, 
            Quaternion velocity, Quaternion angularVelocity,
            double radius, double mass
            )
        {
            Body = this;

            Radius = radius;

            ActiveColor   = Colorf( 0.9f, 0.7f, 0.7f, 0.8f );
            InactiveColor = Colorf( 0.7f, 0.7f, 0.9f, 0.8f );

            SetMass( mass );

            Body->Set_XQVW( position, orientation, velocity, angularVelocity );
            Body->Activate ();
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Gets underlying rigid body.
         */
        virtual RigidBody& GetBody () 
        {
            return *this;
        }

        /** Gets underlying geometry.
         */
        virtual Geometry& GetGeometry () 
        { 
            return *this;
        }

        /** Draws the geometry.
         */
        virtual void Render( RenderType type )
        {
            if ( type == BodyAxes )
            {
                RenderStateVariables( *Body, Quaternion(0,1,1,1) * Radius * 2.0 );
                return;
            }

            GLTransform bodySpace( *Body, /* flattenHeight if */ type == BodyShadow );

            if ( type != BodyShadow )
            {
                if ( Body->IsActive ) {
                    glColor4f( ActiveColor.R, ActiveColor.G, ActiveColor.B,
                        type == BodyShape ? ActiveColor.A : ActiveColor.A/2 );
                }
                else {
                    glColor4f( InactiveColor.R, InactiveColor.G, InactiveColor.B,
                        type == BodyShape ? ActiveColor.A : ActiveColor.A/2 );
                }
            }

            glutSolidSphere( Radius, slices, stacks );
        }

        /** Draws the wireframe of the rigid body.
         */
        virtual void RenderWireframe( double* transform )
        {
            GLTransform bodySpace( transform );
            glColor4f( 0.0f, 0.0f, 0.0f, 0.1f );
            glutWireSphere( Radius, slices, stacks );
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Encapsulates a rigid body with geometry of a rectangular parallelepiped.
     */
    class Box : public Cuboid, public RigidBody, public GLUT_Renderer
    {
    public:

        /////////////////////////////////////////////////////////////////////////////////

        /** Creates the box at the specific location. 
         */
        Box( const Quaternion& position, const Quaternion& orientation,
            const Quaternion& velocity, const Quaternion& angularVelocity,
            const Quaternion& halfExtent, double mass
            )
        {
            Body = this;

            HalfExtent = halfExtent;

            double minsz = HalfExtent.x;
            minsz = HalfExtent.y < minsz ? HalfExtent.y : minsz;
            minsz = HalfExtent.z < minsz ? HalfExtent.z : minsz;

            ActiveColor = minsz < 0.1 ? Colorf( 0.0f, 0.0f, 0.1f, 0.7f )
                                      : Colorf( 0.7f, 0.9f, 0.7f, 0.8f );

            InactiveColor = Colorf( 0.9f, 0.5f, 0.5f, 0.8f );

            SetMass( mass );

            Body->Set_XQVW( position, orientation, velocity, angularVelocity );
            Body->Activate ();
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Gets underlying rigid body.
         */
        virtual RigidBody& GetBody () 
        {
            return *this;
        }

        /** Gets underlying geometry.
         */
        virtual Geometry& GetGeometry () 
        { 
            return *this;
        }

        /** Draws the geometry
         */
        virtual void Render( RenderType type )
        {
            if ( type == BodyAxes )
            {
                RenderStateVariables( *Body, HalfExtent * 1.2 );
                return;
            }

            GLTransform bodySpace( *Body, /* flattenHeight if */ type == BodyShadow );

            if ( type != BodyShadow )
            {
                if ( Body->IsActive ) {
                    glColor4f( ActiveColor.R, ActiveColor.G, ActiveColor.B,
                        type == BodyShape ? ActiveColor.A : ActiveColor.A/2 );
                }
                else {
                    glColor4f( InactiveColor.R, InactiveColor.G, InactiveColor.B,
                        type == BodyShape ? InactiveColor.A : InactiveColor.A/2 );
                }
            }

            glScaled( HalfExtent.x * 2, HalfExtent.y * 2, HalfExtent.z * 2 );
            glutSolidCube( 1.0 );
        }

        /** Draws the wireframe of the rigid body.
         */
        virtual void RenderWireframe( double* transform )
        {
            GLTransform bodySpace( transform );
            glColor4f( 0.0f, 0.0f, 0.0f, 0.1f );
            glScaled( HalfExtent.x * 2, HalfExtent.y * 2, HalfExtent.z * 2 );
            glutWireCube( 1.0 );
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** GLUT wrapper around a single instance of a GlutApplication template class. 
     * GlutApplication should implement event handlers that respond to GLUT callbacks.
     */
    template<class GlutApplication>
    class GLUT_Framework
    {
        /** Indicates whether glut has been initialized.
         */
        bool Initialized;

        /** Holds an instance of the GlutApplication
         */
        static GlutApplication* Application;

        /** Called when glut closes the application window.
         */
        static void CloseFunc ()
        {
            if ( Application && Application->IsValid () ) {
                Application->CloseEventHandler ();
            }
        }

        /** Called each frame to display the scene.
         */
        static void DisplayFunc ()
        {
            if ( Application && Application->IsValid () ) {
                Application->DisplayEventHandler ();
            }
        }

        /** Called when a mouse button is pressed.
         */
        static void MouseFunc( int button, int state, int x, int y )
        {
            if ( Application && Application->IsValid () ) {
                Application->MouseEventHandler( button, state, x, y );
            }
        }

        /** Called when the display window changes size.
         */
        static void ReshapeFunc( int width, int height )
        {
            if ( Application && Application->IsValid () ) {
                Application->ReshapeEventHandler( width, height );
            }
        }

        /** Called when a key is pressed.
         */
        static void KeyboardFunc( unsigned char key, int /*x*/, int /*y*/ )
        {
            if ( Application && Application->IsValid () ) {
                Application->KeyboardEventHandler( key );
            }
        }

        /** Called when a special key is pressed.
         */
        static void SpecialFunc( int key, int /*x*/, int /*y*/ )
        {
            if ( Application && Application->IsValid () ) {
                Application->SpecialKeyEventHandler( key );
            }
        }

        /** Called when the mouse is dragged.
         */
        static void MotionFunc( int x, int y )
        {
            if ( Application && Application->IsValid () ) {
                Application->MotionEventHandler( x, y );
            }
        }

        /** Called when the mouse wheel is spun.
         */
        static void MouseWheelFunc( int wheel, int direction, int x, int y )
        {
            if ( Application && Application->IsValid () ) {
                Application->MouseWheelEventHandler( wheel, direction, x, y );
            }
        }

    public:

        GLUT_Framework ()
            : Initialized( false )
        {
            Printf( "WoRB: GLUT_Framework: Constructed\n" );
        }

        ~GLUT_Framework ()
        {
            Initialized = false;
            Printf( "WoRB: GLUT_Framework: Destructed\n" );
        }

        bool Initialize ()
        {
            // We must have proper dummy argument for glutInit() not to crash.
            int   argc   = 1; 
            char  arg0[] = "WoRB";
            char* argv[] = { arg0 };

            return Initialize( &argc, argv );
        }

        bool Initialize( int* argc, char* argv[] )
        {
            if ( Initialized )  {
                return true;
            }

            Printf( "WoRB: GLUT_Framework: Calling glutInit...\n" );

            // In case of glut >= v2.8.0
            #ifdef GLUT_HAS_MULTI
                glutInitWarningFunc( OnGlutWarning );
                glutInitErrorFunc( OnGlutError );
            #endif

            glutInit( argc, argv );

            Initialized = glutGet( GLUT_INIT_STATE ) != 0;

            return Initialized;
        }

        void Terminate ()
        {
            Disconnect ();

            Printf( "WoRB: GLUT_Framework: glutExit...\n" );
            // glutExit ();

            Initialized = false;
        }

        /** Connects GLUT event handlers to GlutApplication instance.
         */
        void Connect( GlutApplication& application )
        {
            Printf( "WoRB: GLUT_Framework: Connecting event handlers\n" );

            GLUT_Framework::Application = &application;

            glutCloseFunc( CloseFunc );
            glutDisplayFunc( DisplayFunc );
            glutReshapeFunc( ReshapeFunc );
            glutKeyboardFunc( KeyboardFunc );
            glutSpecialFunc( SpecialFunc );
            glutMouseFunc( MouseFunc );
            glutMotionFunc( MotionFunc );
            glutMouseWheelFunc( MouseWheelFunc );
        }

        /** Disconect event handlers from the GlutApplication instance.
         */
        void Disconnect ()
        {
            Printf( "WoRB: GLUT_Framework: Disconnecting event handlers\n" );

            GLUT_Framework::Application = 0;
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////

} // namespace WoRB

#endif // _WORB_UTILITIES_H_INCLUDED
