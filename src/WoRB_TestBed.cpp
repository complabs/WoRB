/**
 *  @file      WoRB_TestBed.cpp
 *  @brief     Implementation of the WoRB_TestBed, a GLUT based WoRB application.
 *  @author    Mikica Kocic
 *  @version   0.3
 *  @date      2012-05-04
 *  @copyright GNU Public License.
 */

#include "WoRB.h"
#include "Utilities.h"
#include "WoRB_TestBed.h"

#include <cstdio>
#include <algorithm> // std::min

using namespace WoRB;

/////////////////////////////////////////////////////////////////////////////////////////
// Destructor. Deallocates all objects.
//
WoRB_TestBed::~WoRB_TestBed ()
{
    for ( RBObjects::iterator i = Objects.begin(); i != Objects.end(); ++i )
    {
        delete *i;
    }

    Objects.clear ();

    IsInitialized = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Runs the simulation and rendering.
//
void WoRB_TestBed::Run ()
{
    Printf( "WoRB: WoRB_TestBed: Run\n" );

    OnProcessData (); // Process initial simulation data
    
    glutPopWindow ();

    while ( IsRunning )
    {
        Simulate ();
        glutMainLoopEvent ();
    }

    glutDestroyWindow( WindowId );
    for ( unsigned i = 0; i < 10; ++i ) {
        glutMainLoopEvent ();
    }

    Printf( "WoRB: Destroyed GLUT window %d\n", WindowId );

    IsInitialized = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Delayed constructor
//
void WoRB_TestBed::Initialize ()
{
    Printf( "WoRB: WoRB_TestBed: Initialize\n" );

    WindowTitle = "Lab4: World of Rigid Bodies";

    // Initialize all properties to default values
    //
    TestSuite            = 0;      // Start test-suite #0
    IsInitialized        = false;  // Not initialized yet
    IsRunning            = true;   // Main-loop ends, when set to false
    IsPaused             = false;  // Is simulation paused: no
    AutoPause            = false;  // Is single-step mode: no
    Wireframe            = false;  // Show bodies in wireframe instead of solid: no
    ShowBodyAxes         = true;   // Show body axes: yes
    ShowFloorMirror      = false;  // Show objects mirrored in the floor: no
    ShowContacts         = false;  // Show contact normals: no
    ShowTrajectories     = false;  // Show object's trajectories: no
    ShowStateVariables   = true;   // Show system state variables: yes
    ShowHelp             = true;   // Show short help: yes
    GridTickLength       = 1.0;    // Tick size of the grid, in meters
    GridTicks            = 50;     // Number of ticks in the grid
    TimeStep             = 0.01;   // Integrator time-step, in seconds
    TimeStepsPerFrame    = 1;      // Number of time-steps solved per one video frame
    TimeStepsPerSnapshot = 20;     // Number of time-steps per one trajectory snapshot
    CameraZoom           = 15.0;   // Position in m, from the coordinate system origin
    CameraLookAt.x       = -2.0;   // Look at x = -2 m
    CameraLookAt.y       = 2.0;    // Look at y = 2 m (height)
    CameraLookAt.z       = 0.0;    // Look at z = 0 m
    CameraAngle          = 55.0;   // Angle in degrees, left/right of x-axis
    CameraElevation      = 25.0;   // Angle in degres, up/bellow the horizon
    FollowObject         = 0;      // Follow the first object (with camera)
    LastDisplayTime      = 0.0;    // Force immediate update
    FinalTime            = 0.0;    // No final (when simulation ends) time
}

/////////////////////////////////////////////////////////////////////////////////////////
// Dumps all parameter values.
//
void WoRB_TestBed::Dump () const
{
    Printf( "IsInitialized        : %s\n",  IsInitialized       ? "true" : "false" );
    Printf( "IsRunning            : %s\n",  IsRunning           ? "true" : "false" );
    Printf( "IsPaused             : %s\n",  IsPaused            ? "true" : "false" );
    Printf( "AutoPause            : %s\n",  AutoPause           ? "true" : "false" );
    Printf( "Wireframe            : %s\n",  Wireframe           ? "true" : "false" );
    Printf( "ShowBodyAxes         : %s\n",  ShowBodyAxes        ? "true" : "false" );
    Printf( "ShowFloorMirror      : %s\n",  ShowFloorMirror     ? "true" : "false" );
    Printf( "ShowContacts         : %s\n",  ShowContacts        ? "true" : "false" );
    Printf( "ShowTrajectories     : %s\n",  ShowTrajectories    ? "true" : "false" );
    Printf( "ShowStateVariables   : %s\n",  ShowStateVariables  ? "true" : "false" );
    Printf( "ShowHelp             : %s\n",  ShowHelp            ? "true" : "false" );

    Printf( "GridTickLength       : %g m\n", GridTickLength       );
    Printf( "GridTicks            : %d\n",   GridTicks            );

    Printf( "TimeStep             : %g s\n", TimeStep             );
    Printf( "TimeStepsPerFrame    : %u\n",   TimeStepsPerFrame    );
    Printf( "TimeStepsPerSnapshot : %u\n",   TimeStepsPerSnapshot );
    Printf( "FinalTime            : %g s\n", FinalTime            );

    Printf( "FollowObject         : %lu\n",  FollowObject         );
    Printf( "CameraAngle          : %g°\n",  CameraAngle          );
    Printf( "CameraElevation      : %g°\n",  CameraElevation      );
    Printf( "CameraZoom           : %g m\n", CameraZoom           );
    Printf( "CameraLookAt         : [ %g, %g, %g ] m\n", 
        CameraLookAt.x, CameraLookAt.y, CameraLookAt.z );

    // Now display objects in the system
    //
    for ( unsigned i = 0; i < Objects.size (); ++i )
    {
        const Geometry&  g = Objects.at(i)->GetGeometry ();
        const RigidBody& b = Objects.at(i)->GetBody ();

        Printf( "\nObject %d\n", i + 1 );

        Printf( "Geometry         : %s\n", g.GetName () );

        if ( g.IsSphere () )
        {
            const Sphere& s = (const Sphere&)g;
            Printf( "Radius           : %g m\n", s.Radius );
        }
        else if ( g.IsCuboid () )
        {
            const Cuboid& c = (const Cuboid&)g;
            Printf( "Half-Extent      : [ %g, %g, %g ] m\n", 
                c.HalfExtent.x, c.HalfExtent.y, c.HalfExtent.z );
        }

        Printf( "Mass             : %g kg\n", b.Mass () );

        Printf( "Position         : [ %g, %g, %g | %g ] m\n", 
            b.Position.x, b.Position.y, b.Position.z, b.Position.w );

        Printf( "Orientation      : [ %g, %g, %g | %g ]\n", 
            b.Orientation.x, b.Orientation.y, b.Orientation.z, b.Orientation.w );

        Printf( "Linear Momentum  : [ %g, %g, %g | %g ] kg m s^-1\n", 
            b.LinearMomentum.x, b.LinearMomentum.y, 
            b.LinearMomentum.z, b.LinearMomentum.w );

        Printf( "Angular Momentum : [ %g, %g, %g | %g ] kg m^2 s^-1m\n", 
            b.AngularMomentum.x, b.AngularMomentum.y, 
            b.AngularMomentum.z, b.AngularMomentum.w );

        Printf( "Velocity         : [ %g, %g, %g | %g ] m s^-1\n", 
            b.Velocity.x, b.Velocity.y, b.Velocity.z, b.Velocity.w );

        Printf( "Angular Velocity : [ %g, %g, %g | %g ] s^-1\n", 
            b.AngularVelocity.x, b.AngularVelocity.y, 
            b.AngularVelocity.z, b.AngularVelocity.w );

        Printf( "Kinetic Energy   : %g J\n", b.KineticEnergy );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Creates the GLUT window and initializes the view.
//
void WoRB_TestBed::SetupAnimation ()
{
    // Setup common execution options for GLUT: not to call exit() when finished
    //
    glutSetOption( GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION );

    // Create GLUT window
    //
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
    glutInitWindowSize( 800, 600 );
    
    WindowId = glutCreateWindow( WindowTitle );

    glutForegroundWindow ();

    Printf( "WoRB: Created GLUT window %d\n", WindowId );

    // Initialize ambient light
    //
    GLfloat lightAmbient[] = { 0.5, 0.5, 0.5, 1 };
    glLightfv( GL_LIGHT0, GL_AMBIENT, lightAmbient );

    GLfloat lightDiffuse[] = { 1, 1, 1, 1 };
    glLightfv( GL_LIGHT0, GL_DIFFUSE, lightDiffuse );

    glEnable( GL_LIGHT0 );

    glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );
    glEnable( GL_DEPTH_TEST );
    glShadeModel( GL_SMOOTH );

    // Finally, setup camera position
    //
    SetupProjection ();

    // .. and mark the instance as initialized.
    //
    IsInitialized = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Sets the projection characteristics of the camera.
//
void WoRB_TestBed::SetupProjection ()
{
    double aspect = double( glutGet( GLUT_WINDOW_WIDTH ) ) 
                    / std::max( 1, glutGet( GLUT_WINDOW_HEIGHT ) );

    aspect = std::min( 2e3, std::max( -2e3, aspect ) );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity ();
    gluPerspective( 45.0, aspect, 1.0, 500.0 );
    glMatrixMode( GL_MODELVIEW );
}

/////////////////////////////////////////////////////////////////////////////////////////
// Updates the current state (solves ODE) of the system.
//
void WoRB_TestBed::Simulate ()
{
    // Reconfigure the test-bed, if requested
    //
    if ( TestSuite >= 0 )
    {
        ReconfigureTestBed ();
        TestSuite = -1; // Set the flag to 'initialized'
    }

    // Just refresh display and sleep some time, when paused
    //
    if ( IsPaused )
    {
        glutPostRedisplay ();

        // Just wait
        double durationMs = TimeStep * TimeStepsPerFrame * 1e3; // in milliseconds
        Pause( (unsigned long)durationMs );

        return;
    }

    // If not paused, solve ODE
    //
    worb.SolveODE( TimeStep );

    // Process data calculated during the simulation, e.g. save data.
    //
    OnProcessData ();

    if ( FinalTime > 0 && worb.Time >= FinalTime )
    {
        IsRunning = false;
    }

    // Take a snapshot of object's positions, when snapshot time comes
    //
    if ( worb.TimeStepCount % TimeStepsPerSnapshot == 0 && ShowTrajectories )
    {
        for ( RBObjects::iterator i = Objects.begin(); i != Objects.end(); ++i )
        {
            if ( (*i)->ShowTrajectory ) {
                TrajectoryItem ti;
                ti.object = *i;
                (*i)->GetBody ().ToWorld.GetGLTransform( ti.matrix );
                Trajectories.push_back( ti );
            }
        }
    }

    // Animate objects, when (animation frame) time comes 
    //
    if ( worb.TimeStepCount % TimeStepsPerFrame == 0 || AutoPause )
    {
        glutPostRedisplay ();
    }

    // Clear auto-pause flag, i.e. force user to set `IsPaused = false` 
    // and `AutoPause = true` for the next simulation single-step.
    //
    if ( AutoPause )
    {
        AutoPause = false;
        IsPaused = true;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Renders the current scene.
//
void WoRB_TestBed::DisplayEventHandler ()
{
    // Adjust camera's 'look-at' position depending on the selected object to follow
    //
    if ( FollowObject < Objects.size () )
    {
         CameraLookAt = Objects.at( FollowObject )->GetBody ().Position;
    }

    // Clear the viewport and setup the camera direction
    //
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity ();

    gluLookAt( /*eye*/ CameraZoom, 0, 0, /*center*/ 0, 0, 0, /*up*/ 0, 1, 0 );

    glTranslated ( -CameraZoom,      0, 0    ); // move away = zoom out
    glRotated    ( -CameraElevation, 0, 0, 1 ); // z-rotate = elevation
    glRotated    (  CameraAngle,     0, 1, 0 ); // y-rotate = angle

    glTranslated ( -CameraLookAt.x, -CameraLookAt.y, -CameraLookAt.z );

    if ( CameraElevation >= -8.0 )
    {
        // Render objects reflected in the floor (mirror)
        //
        if ( ShowFloorMirror )
        {
            glEnable( GL_DEPTH_TEST );
            glEnable( GL_LIGHTING );
            glEnable( GL_BLEND );

            glColorMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE );
            glEnable( GL_COLOR_MATERIAL );

            static const GLfloat lightPosition[] = { 1, -1, 0, 0 };
            glLightfv( GL_LIGHT0, GL_POSITION, lightPosition );

            // A transform matrix for rendering objects reflected in the floor
            //
            static const GLdouble floorMirrorTransform [] =
            {
                1,  0, 0, 0,
                0, -1, 0, 0,
                0,  0, 1, 0,
                0,  0, 0, 1
            };

            glPushMatrix();
            glMultMatrixd( floorMirrorTransform );

            for ( RBObjects::iterator i = Objects.begin(); i != Objects.end(); ++i )
            {
                (*i)->Render( GLUT_Renderer::FloorMirror );
            }

            glPopMatrix();
        }

        glDisable( GL_COLOR_MATERIAL );
        glDisable( GL_LIGHTING );
        glDisable( GL_DEPTH_TEST );
        glDisable( GL_BLEND );
    }

    // Render a tiled ground plane i.e. the xz-grid
    //
    glColor3d( 0.95, 0.95, 0.85 );

    glBegin( GL_LINES );
    for ( int x = -GridTicks; x <= GridTicks; ++x )
    {
        for ( int z = -GridTicks; z <= GridTicks; ++z ) 
        {
            glVertex3d(  x * GridTickLength, 0, -z * GridTickLength );
            glVertex3d(  x * GridTickLength, 0,  z * GridTickLength );
            glVertex3d(  x * GridTickLength, 0,  z * GridTickLength );
            glVertex3d( -x * GridTickLength, 0,  z * GridTickLength );
        }
    }
    glEnd ();

    // Display the main axes
    //
    RenderAxes( 10 * GridTickLength );

    // Render ground shadows of the objects
    //
    glEnable( GL_BLEND );
    glDisable( GL_DEPTH_TEST );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    glColor4d( 0.1, 0.1, 0, 0.1 ); 
    for ( RBObjects::iterator i = Objects.begin(); i != Objects.end(); ++i )
    {
        (*i)->Render( GLUT_Renderer::BodyShadow );
    }

    // Render the objects themselves
    //
    glEnable( GL_LIGHTING );

    static const GLfloat lightPositionForMirror [] = { 1, 1, 0, 0 };
    glLightfv( GL_LIGHT0, GL_POSITION, lightPositionForMirror );

    glColorMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE );
    glEnable( GL_COLOR_MATERIAL );

    if ( Wireframe ) {
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    }
    else {
        glEnable( GL_DEPTH_TEST );
    }

    for ( RBObjects::iterator i = Objects.begin(); i != Objects.end(); ++i )
    {
        (*i)->Render( GLUT_Renderer::BodyShape );
    }

    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    glDisable( GL_COLOR_MATERIAL );
    glDisable( GL_LIGHTING );

    // Render trajectories
    //
    if ( ShowTrajectories )
    {
        for ( Trajectory::iterator i = Trajectories.begin(); i != Trajectories.end(); ++i )
        {
            (*i).object->RenderWireframe( (*i).matrix );
        }
    }

    glDisable( GL_DEPTH_TEST );
    glDisable( GL_BLEND );

    // Render any additional information

    RenderDebugInfo ();

    // Update the display and swap double-buffers
    //
    glFlush ();
    glutSwapBuffers ();

    // Remember the current time
    //
    double currentTime = glutGet( GLUT_ELAPSED_TIME );

    double durationMs = TimeStep * TimeStepsPerFrame * 1e3; // in milliseconds
    durationMs -= currentTime - LastDisplayTime;

    if ( durationMs > 0 ) {
        Pause( (unsigned long)durationMs );
    }

    LastDisplayTime = currentTime;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Renders state variables and short help.
//
void WoRB_TestBed::RenderDebugInfo ()
{
    // Draw axes of the body
    //
    if ( ShowBodyAxes )
    {
        for ( RBObjects::iterator i = Objects.begin(); i != Objects.end(); ++i )
        {
            (*i)->Render( GLUT_Renderer::BodyAxes );
        }
    }

    // Display the state variables of the system
    //
    if ( ShowStateVariables )
    {
        GLOrthoScreen _inScreenCoordinates; // Establish temporary transform for text

        glColor3d( 0, 0, 0.7 );

        const double E_k = worb.TotalKineticEnergy;
        const double E_p = worb.TotalPotentialEnergy;
        const Quaternion& p_tot = worb.TotalLinearMomentum;
        const Quaternion& L_tot = worb.TotalAngularMomentum;

        int row = glutGet( GLUT_WINDOW_HEIGHT ) - 20;
        row = RenderPrintf( 10, row, 
            "N = %4lu, t = %6.3lf%s\n"
            "E_t/k/p %12.3lf %12.3lf %12.3lf\n"
            "p_tot   %12.3lf %12.3lf %12.3lf\n"
            "L_tot   %12.3lf %12.3lf %12.3lf",
            worb.TimeStepCount, worb.Time,
            IsPaused || AutoPause ? " (Paused)" : "",
            E_k + E_p, E_k, E_p,
            p_tot.x, p_tot.y, p_tot.z,
            L_tot.x, L_tot.y, L_tot.z
        );

        // Display the state variables of the rigid bodies
        //
        unsigned nShown = unsigned( Objects.size () );
        nShown = std::min( IsPaused || AutoPause ? 4u : 0u, nShown );
        for ( unsigned i = 0; i < nShown; ++i )
        {
            const RigidBody&  b = Objects.at(i)->GetBody ();
            const Quaternion& x = b.Position;
            const Quaternion& q = b.Orientation;
            const Quaternion& p = b.LinearMomentum;
            const Quaternion& L = b.AngularMomentum;
            const Quaternion& v = b.Velocity;
            const Quaternion& w = b.AngularVelocity;

            row = RenderPrintf( 10, row, 
                "(%d)   x %12.3lf %12.3lf %12.3lf\n"
                "      q %12.3lf %12.3lf %12.3lf %12.3lf\n"
                "      p %12.3lf %12.3lf %12.3lf\n"
                "      L %12.3lf %12.3lf %12.3lf\n"
                "      v %12.3lf %12.3lf %12.3lf\n"
                "      w %12.3lf %12.3lf %12.3lf",
                i + 1, x.x, x.y, x.z,
                       q.x, q.y, q.z, q.w,
                       p.x, p.y, p.z,
                       L.x, L.y, L.z,
                       v.x, v.y, v.z,
                       w.x, w.y, w.z
            );
        }
    }

    // Display short help
    //
    if ( ShowHelp )
    {
        GLOrthoScreen _inScreenCoordinates; // Establish temporary transform for text

        glColor3d( 0, 0, 0 );
        RenderPrintf( 10, 4 * 25, 
            "Shortcut keys:\n"
            "  1, 2, ... for different simulation\n"
            "  (P)ause, (S)ingle-step, (Q)uit\n"
            "  (A)xes, (V)ariables, (C)ontacts, (T)rajectories\n"
            "  (W)ireframe, Floor (M)irror, (F)ullscreen"
        );
        glColor3d( 0, 0, 1 );
        RenderPrintf( 10, 10, 
            "Camera: a= %+5.1lf, e= %+5.1lf, d= %+5.1lf, at= %+5.1lf %+5.1lf %+5.1lf",
            CameraAngle, CameraElevation, CameraZoom,
            CameraLookAt.x, CameraLookAt.y,  CameraLookAt.z
        );
    }

    // Display contact normals. 
    // Green between two objects, red between objects and scenery e.g. floor.
    //
    if ( ShowContacts )
    {
        glLineWidth( 3 );
        glBegin( GL_LINES );

        for ( unsigned i = 0; i < worb.Collisions.Count (); ++i )
        {
            Quaternion pos = worb.Collisions[i].Position;
            Quaternion n = worb.Collisions[i].Normal;
            Quaternion end = pos + n;

            if ( worb.Collisions[i].WithScenery () ) {
                glColor3d( 1, 0, 0 ); // red body
            } else {
                glColor3d( 0, 1, 0 ); // green body
            }

            glVertex3d( pos.x, pos.y, pos.z );
            glVertex3d( end.x, end.y, end.z );

            pos = end;
            end = pos + n * 0.1;

            glColor3d( 0, 0, 1 ); // blue head
            glVertex3d( pos.x, pos.y, pos.z );
            glVertex3d( end.x, end.y, end.z );
        }

        glEnd ();
        glLineWidth( 1 );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Called when GLUT detects a mouse drag.
//
void WoRB_TestBed::MotionEventHandler( int x, int y )
{
    int modifiers = glutGetModifiers ();

    if ( modifiers == GLUT_ACTIVE_CTRL && LastMouse.state == GLUT_DOWN )
    {
        CameraZoom += 0.5 * ( y - LastMouse.y );
        CameraZoom = std::max( 0.5, std::min( 300.0, CameraZoom ) );
    }
    else if ( modifiers == GLUT_ACTIVE_SHIFT && LastMouse.state == GLUT_DOWN )
    {
        double k  = CameraZoom * 2e-3;
        double dx = k * ( x - LastMouse.x );
        double dy = k * ( y - LastMouse.y );

        double phi   = CameraAngle     * Const::Pi / 180.0;
        double theta = CameraElevation * Const::Pi / 180.0;

        CameraLookAt.y +=  dy * cos( theta );
        CameraLookAt.x += -dx * sin( phi ) - dy * cos( phi ) * sin( theta );
        CameraLookAt.z +=  dx * cos( phi ) - dy * sin( phi ) * sin( theta );

        if ( CameraLookAt.y < 0 ) {
            CameraLookAt.y = 0;
        }
    }
    else
    {
        CameraAngle += 0.25 * ( x - LastMouse.x );
        while( CameraAngle < -180.0 ) {
            CameraAngle += 360.0;
        }
        while ( CameraAngle > 180.0 ) {
            CameraAngle -= 360.0;
        }

        CameraElevation += 0.25 * ( y - LastMouse.y );
        CameraElevation = std::max( -20.0, std::min( 90.0, CameraElevation ) );
    }

    // Remember the current mouse position
    //
    LastMouse.x = x;
    LastMouse.y = y;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Called when GLUT detects a key press.
//
void WoRB_TestBed::KeyboardEventHandler( unsigned char key )
{
    switch( key )
    {
        case 'A': case 'a': // Toggle displaying axes of the rigid bodies
            ShowBodyAxes = ! ShowBodyAxes;
            break;

        case 'C': case 'c': // Toggle displaying contact info
            ShowContacts = ! ShowContacts;
            break;

        case 'F': case 'f': // Toggle fullscreen mode
            glutFullScreenToggle ();
            break;

        case 'H': case 'h': // Toggle help mode
            ShowHelp = ! ShowHelp;
            break;

        case 'M': case 'm': // Toggle floor mirror
            ShowFloorMirror = ! ShowFloorMirror;
            break;

        case 'Q': case 'q': // Quit application
            IsRunning = false;
            break;

        case 'P': case 'p': case ' ': // Toggle running the simulation
            IsPaused = ! IsPaused;
            break;

        case 'S': case 's': case '\r': // Advance only one time-step
            AutoPause = true;
            IsPaused = false;
            break;

        case 'T': case 't': // Toggle displaying trajectories
            ShowTrajectories = ! ShowTrajectories;
            // Trajectories.clear ();
            break;

        case 'V': case 'v': // Toggle displaying state variables
            ShowStateVariables = ! ShowStateVariables;
            break;

        case 'W': case 'w': // Toggle wireframe mode
            Wireframe = ! Wireframe;
            break;

        case '1': case '2': case '3': case '4': 
        case '5': case '6': case '7': case '8': case '9':
            TestSuite = key - '1';
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Called when GLUT detects a function key press.
//
void WoRB_TestBed::SpecialKeyEventHandler( int key )
{
    switch( key )
    {
        case GLUT_KEY_F1: // Follow body 1
            FollowObject = 0;
            break;

        case GLUT_KEY_F2: // Follow body 2
            FollowObject = 1;
            break;

        case GLUT_KEY_F3: // Follow body 3
            FollowObject = 2;
            break;

        case GLUT_KEY_F4: // Follow body 4
            FollowObject = 3;
            break;

        case GLUT_KEY_F11: // Look at coordinate origin, with offset of Oxyz bisectris
            FollowObject    = 0xFFFFu;
            CameraLookAt    = 0.0;
            CameraAngle     = 55.0;
            CameraElevation = 25.0;
            CameraZoom      = 20;
            break;

        case GLUT_KEY_F12: // Look at coordinate origin, from above
            FollowObject    = 0xFFFFu;
            CameraLookAt    = 0.0;
            CameraAngle     = 0.0;
            CameraElevation = 90.0;
            CameraZoom      = 30;
            break;

    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Clears the current simulation data and prepares a new simulation.
//
void WoRB_TestBed::ClearTestBed ()
{
    // Remove rigid body references from the WoRB solver
    //
    worb.RemoveObjects ();

    // Setup default parameters for collision detection/resolve algorithms
    //
    worb.Collisions.Restitution = 1;    // Coefficient of restitution
    worb.Collisions.Relaxation  = 0.2;  // Position projection relaxation coefficient
    worb.Collisions.Friction    = 0;    // Dynamic friction coefficient

    // Disable gravity
    //
    worb.Gravity = 0.0;

    // Initialize the ground plane and the walls
    //
    double boxHalfSize = GridTicks * GridTickLength;
    GroundPlane.Direction =  Const::Y;
    GroundPlane.Offset    =  0.0;
    BoxWall[0].Direction  =  Const::X; // left
    BoxWall[0].Offset     = -boxHalfSize;
    BoxWall[1].Direction  = -Const::X; // right
    BoxWall[1].Offset     = -boxHalfSize;
    BoxWall[2].Direction  =  Const::Z; // rear
    BoxWall[2].Offset     = -boxHalfSize;
    BoxWall[3].Direction  = -Const::Z; // front
    BoxWall[3].Offset     = -boxHalfSize;

    // Add the ground plane and the walls to our WoRB solver
    //
    worb.Add( GroundPlane );

    // for ( unsigned i = 0; i < 4; ++i ) {
    //     worb.Add( BoxWall[i] );
    // }

    // Remove existing trajectories
    //
    Trajectories.clear ();
    Trajectories.reserve( 10000 );

    // Remove existing objects
    //
    for ( RBObjects::iterator i = Objects.begin(); i != Objects.end(); ++i ) {
        delete *i;
    }

    Objects.clear ();
    Objects.reserve( 64 );
}

/////////////////////////////////////////////////////////////////////////////////////////
// Initializes the test-bed with the default objects according to selected profile.
//
void WoRB_TestBed::ReconfigureTestBed ()
{
    /////////////////////////////////////////////////////////////////////////////////////

    ClearTestBed ();       // Clear existing simulation, if any.
    LastDisplayTime = 0;   // Force display refresh

    if ( TestSuite >= 6 ) {
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////

    /* LAB4 */

    ShowBodyAxes = true;

    double thick = 0.01;
    double v = -1;
    double mass = 0.1;
    double L = 5.0;

    if ( TestSuite >= 1 )
    {
        thick = 0.7;
        v = -20;
        mass = 10e3;
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Add new objects

    Box* box1 = new Box(
        /*x=*/ SpatialVector( -L/2, 3, 0 ), 
        /*q=*/ Quaternion::FromAxisAngle( Const::Pi/2, 0, 1, 0 ),
        /*v=*/ 0.0, /*w=*/ 0.0,
        /*extent=*/ SpatialVector( L, thick, L/2 ), /*mass=*/ mass
    );
    Objects.push_back( box1 );
    worb.Add( box1 );

    /////////////////////////////////////////////////////////////////////////////////////

    Box* box2 = new Box(
        /*x*/ SpatialVector( L - v, 3, L/2 ), /*q=*/ Quaternion( 0, 0, 1, 0 ),
        /*v*/ v * Const::X, /*w=*/ 0.0,
        /*extent=*/ SpatialVector( L, thick, L/2 ), /*mass=*/ mass
    );

    Objects.push_back( box2 );
    worb.Add( box2 );

    /////////////////////////////////////////////////////////////////////////////////////

    if ( TestSuite >= 1 ) 
    {
        // box2->Orientation.w -= 1e-4;
        box2->Orientation.w += 1e-4;

        box1->RigidBody::Position.y += 1.0;
        box2->RigidBody::Position.y += 1.01;
    }

    /////////////////////////////////////////////////////////////////////////////////////

    if ( TestSuite >= 2 && TestSuite <= 3 )
    {
        for ( int i = 0; i < 30; ++i ) {
        Ball* ball = new Ball(
            RandomQuaternion( SpatialVector( 1, 3, 0 ), SpatialVector( 1, 20, 0 ) ),
            RandomQuaternion (), 
            /*v=*/ 0.0, /*w=*/ 0.0, /*r=*/ 0.5, /*mass=*/ 1e1
        );

        Objects.push_back( ball );
        worb.Add( ball );
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////

    if ( TestSuite >= 2 )
    {
        worb.Gravity = Const::g_n; // add gravity

        ShowBodyAxes = false;

        box1->SetMass( 3 );
        box1->Body->Position.y = 5;
        box1->Body->CanBeDeactivated = true;

        box2->Body->Position.y = 5;
        box2->Body->CanBeDeactivated = true;
    }

    if ( TestSuite >= 3 )
    {
        worb.Collisions.Restitution = 0.2;
        worb.Collisions.Friction = 0.2;
    }

    if ( TestSuite >= 4 )
    {
        ShowBodyAxes = false;

        for ( int i = 0; i < 50; ++i )
        {
            Box* box;
            if ( TestSuite >= 5 ) {
                box2->Body->Velocity *= 0.8;
                box2->Body->CalculateDerivedQuantities( false );

                box = new Box(
                    /*x*/ SpatialVector( L, i * 0.4 + 0.2, L/2 ), 
                    /*q*/ Quaternion( 1.0 ),
                    /*v*/ 0.0, /*w*/ 0.0,
                    /*extent=*/ SpatialVector( 2, 0.2, 2 ),
                    /*mass=*/ mass
                );
            }
            else {
                box2->Body->Velocity = 0.0;
                box2->Body->CalculateDerivedQuantities( false );
                worb.Collisions.Relaxation = 0.0;

                box = new Box(
                    /*x*/ SpatialVector( L, i * 0.4 + 0.2, L/2 ), 
                    /*q*/ RandomQuaternion (),
                    /*v*/ 0.0, /*w*/ 0.0,
                    /*extent=*/ RandomQuaternion(
                        SpatialVector( 0.5, 0.5, 0.5 ), SpatialVector( 1, 2, 3 ) ),
                    /*mass=*/ mass
                );

                box->ActiveColor = RandomQuaternion ();
                box->ActiveColor.A = 0.8f;
            }
            box->Body->CanBeDeactivated = true;

            Objects.push_back( box );
            worb.Add( box );
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Finally, initialize ODE (calculate derived quantities for recently added objects)

    worb.InitializeODE ();
}
