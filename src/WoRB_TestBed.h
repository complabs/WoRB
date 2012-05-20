#ifndef _WORB_TESTBED_H_INCLUDED
#define _WORB_TESTBED_H_INCLUDED

/**
 *  @file      WoRB_TestBed.h
 *  @brief     Definitions for the WoRB_TestBed, which encapsulates
 *             a GLUT based test-bed application based on the WoRB framework.
 *  @author    Mikica Kocic
 *  @version   0.4
 *  @date      2012-05-10
 *  @copyright GNU Public License.
 */

#include "WoRB.h"
#include "Utilities.h"

#include <vector> // for GLUT_Renderer collection

/////////////////////////////////////////////////////////////////////////////////////////

/** Encapsulates a rigid body test-bed application.
 */
class WoRB_TestBed
{
protected:

    /////////////////////////////////////////////////////////////////////////////////////

    /** Holds the WoRB physics simulation framework.
     */
    WoRB::WorldOfRigidBodies<256,1024> worb;

    /** Final simulation time, s.
     */
    double FinalTime;

    /////////////////////////////////////////////////////////////////////////////////////

    typedef std::vector<WoRB::GLUT_Renderer*> RBObjects;

    /** Holds the collection of all rendered rigid bodies in the system.
     */
    RBObjects Objects;

    /** Holds the ground plane, with geometry of a half-space.
     */
    WoRB::HalfSpace GroundPlane;

    /** Holds the box walls, with geometry of half-spaces.
     */
    WoRB::HalfSpace BoxWall[4];

    /** Holds the length of a single tick in the system grid.
     */
    double GridTickLength;

    /** Holds the number of the ticks in the system grid (bounded by the walls).
     */
    int GridTicks;

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents trajectory snapshot item (position and orientation of the rigid body).
     * Contains GLUT_Renderer object (with WoRB geometry and rigid body info) and 
     * current GL transform matrix at snapshot time.
     */
    struct TrajectoryItem {
        WoRB::GLUT_Renderer* object;
        double matrix[ 16 ];
    };

    /** Represents a collection of trajectory snapshots.
     */
    typedef std::vector<TrajectoryItem> Trajectory;

    /** Holds the collection of trajectory snapshots of the rigid bodies in the system.
     */
    Trajectory Trajectories;

    /////////////////////////////////////////////////////////////////////////////////////

    /** Indicates whether the instance is properly constructed.
     */
    volatile bool IsInitialized;
    
    /** Holds window title.
     */
    const char* WindowTitle;

    /** Holds GLUT window ID.
     */
    int WindowId;

    /** Indicates whether the application is running.
     */
    volatile bool IsRunning;

    /** Indicates whether the simulation is running (alternative to being paused).
     */
    bool IsPaused;

    /** Indicates whether the simulation halts after every time-step.
     */
    bool AutoPause; 

    /** Indicates whether to display objects in wireframe.
     */
    bool Wireframe;

    /** Indicates whether to display axes of the rigid bodies.
     */
    bool ShowBodyAxes;

    /** Indicates whether to show objects mirrored in the floor.
     */
    bool ShowFloorMirror;

    /** Indicates whether to display contact normals.
     */
    bool ShowContacts;

    /** Indicates whether to display trajectories.
     */
    bool ShowTrajectories;

    /** Indicates whether to display the system state variables.
     */
    bool ShowStateVariables;

    /** Indicates whether to display help info.
     */
    bool ShowHelp;

    /** Holds the index of an object that is followed by the camera view.
     */
    unsigned FollowObject;

    /** Holds integrator time-step length, in seconds.
     */
    double TimeStep;

    /** Holds number of integrator time-steps solved per one video frame.
     */
    unsigned TimeStepsPerFrame;

    /** Holds number of integrator time-steps solved per one trajectory snapshot.
     */
    unsigned TimeStepsPerSnapshot;

    /** Holds the camera zoom (distance from the look-at point).
     */
    double CameraZoom;

    /** Holds the point camera looks at.
     */
    WoRB::Quaternion CameraLookAt;

    /** Holds the camera angle (phi).
     */
    double CameraAngle;

    /** Holds the camera elevation (theta).
     */
    double CameraElevation;

    /** Holds the position of the mouse at the last frame of a drag.
     */
    struct { double x, y; int button, state; } LastMouse;

    /** Holds time-stamp of the last screen update
     */
    double LastDisplayTime;

    /** Holds next requested configuration profile.
     */
    volatile int TestSuite;

    /////////////////////////////////////////////////////////////////////////////////////

public:

    /** Dummy constructor. Use Initialize() to construct an instance.
     */
    WoRB_TestBed () 
    {
        IsInitialized = false;
    }

    /** Deallocates all objects.
     */
    ~WoRB_TestBed ();

    /** Delayed constructor; after all static members are initialized.
     */
    void Initialize ();

    /** Dumps all parameter values using Printf().
     */
    void Dump () const;

    /** Runs the simulation and rendering.
     */
    void Run ();

    /** Clears the current simulation data and prepares a new simulation.
      */
    void ClearTestBed ();

    /** Initializes the test-bed with the default objects according to selected profile.
      */
    void ReconfigureTestBed ();

    /** Updates the current state of the scene.
     */
    void Simulate ();

    /** Processes data calculated, solved and derived during the simulation time-step.
     * This virtual method can be used to save simulation data in files
     * or to return simulated data to MATLAB for example.
     */
    virtual void OnProcessData () {}

    /** Renders the current scene
     */
    void DisplayEventHandler ();

    /** Displays debugging information (like state variables) and short help.
     */
    void RenderDebugInfo ();

    /** Returns whether application is properly initialized
     */
    bool IsValid () const
    {
        return IsInitialized;
    }

    /** Creates the GLUT window and initializes the view.
     */
    void SetupAnimation ();

    /** Sets the projection characteristics of the camera.
     */
    void SetupProjection ();

    /** Handles on-window-close event.
     */
    void CloseEventHandler ()
    {
        IsRunning = false;
    }

    /** Handles event when the window has changed size.
     */
    void ReshapeEventHandler( int width, int height )
    {
        glViewport( 0, 0, width, height );
        SetupProjection ();
    }

    /** Called when GLUT detects a key press.
     */
    void KeyboardEventHandler( unsigned char key );

    /** Called when GLUT detects a function key press.
     */
    void SpecialKeyEventHandler( int key );

    /** Called when GLUT detects a mouse drag.
     */
    void MotionEventHandler( int x, int y );

    /** Called when GLUT detects a mouse button press.
     */
    void MouseEventHandler( int button, int state, int x, int y )
    {
        // Remember the current mouse state and position
        //
        LastMouse.button = button;
        LastMouse.state = state;
        LastMouse.x = x;
        LastMouse.y = y;
    }

    /** Called when GLUT detects a mouse wheel is spun
     */
    void MouseWheelEventHandler( int /*wheel*/, int direction, int x, int y )
    {
        CameraZoom -= direction;
        CameraZoom = std::max( 0.01, std::min( 200.0, CameraZoom ) );

        // Remember the current mouse position
        //
        LastMouse.x = x;
        LastMouse.y = y;
    }
};

#endif // _WORB_TESTBED_H_INCLUDED
