/**
 *  @file      mexFunction.cpp
 *  @brief     The main entry point for the WoRB MATLAB function.
 *  @author    Mikica Kocic
 *  @version   0.5
 *  @date      2012-05-20
 *  @copyright GNU Public License.
 */

#include "WoRB.h"
#include "Utilities.h"
#include "WoRB_TestBed.h"

#include "mex.h"

#include "mexWoRB.h"

/////////////////////////////////////////////////////////////////////////////////////////

/** Represents WoRB test-bed that can be initialized from `mexFunction` arguments.
 */
class WoRB_MexFunction : public WoRB_TestBed
{
    /** Holds the results returned to MATLAB.
     */
    Mex::Matrix Result;

public:

    /** Default constructor. Creates an empty result matrix.
     */
    WoRB_MexFunction ()
        : Result( 0, 0 )
    {
    }

    /** Initializes parameters and creates bodies from mexFunction arguments.
     * The first mex function argument should be a structure with fields as sys params.
     * The second mex function argument should be a structure array with body parameters.
      */
    void Parse( 
        int nArgIn, const mxArray* argIn[] // right-hand side (varargin) of the function
        )
    {
        if ( nArgIn >= 1 && ! mxIsStruct( argIn[0] ) ) {
            WoRB::SevereError( "WoRB:Init:invarg",
                "The first argument must be a structure with system parameters." );
        }

        if ( nArgIn >= 2 && ! mxIsStruct( argIn[1] ) ) {
            WoRB::SevereError( "WoRB:Init:invarg", 
                "The second argument must be a structure array of body parameters." );
        }

        /////////////////////////////////////////////////////////////////////////////////
        // Clear objects from previous simulations and make default parameters.
        //
            
        Result = Mex::Matrix( 0, 0 );  // Clear earlier results
        Initialize ();                 // Initialize default parameters
        ClearTestBed ();               // Clear existing simulation, if any.

        /////////////////////////////////////////////////////////////////////////////////
        // Parse params structure
        //
        bool dumpInitialState = false;

        if ( nArgIn >= 1 )  
        {
            const mxArray* params = argIn[0];

            WindowTitle = Mex::String( params, 0, "Title" );

            TestSuite            = Mex::Logical( params, 0, "TestSuite"          );
            IsInitialized        = Mex::Logical( params, 0, "IsInitialized"      );
            IsRunning            = Mex::Logical( params, 0, "IsRunning"          );
            IsPaused             = Mex::Logical( params, 0, "IsPaused"           );
            AutoPause            = Mex::Logical( params, 0, "AutoPause"          );
            Wireframe            = Mex::Logical( params, 0, "Wireframe"          );
            ShowBodyAxes         = Mex::Logical( params, 0, "ShowBodyAxes"       );
            ShowFloorMirror      = Mex::Logical( params, 0, "ShowFloorMirror"    );
            ShowContacts         = Mex::Logical( params, 0, "ShowContacts"       );
            ShowTrajectories     = Mex::Logical( params, 0, "ShowTrajectories"   );
            ShowStateVariables   = Mex::Logical( params, 0, "ShowStateVariables" );
            ShowHelp             = Mex::Logical( params, 0, "ShowHelp"           );
            dumpInitialState     = Mex::Logical( params, 0, "DumpInitialState"   );

            GridTickLength = Mex::Scalar( params, 0, "GridTickLength" );
            GridTicks      = int( Mex::Scalar( params, 0, "GridTicks" ) );

            TimeStep             = Mex::Scalar( params, 0, "TimeStep" );
            TimeStepsPerFrame    = unsigned( Mex::Scalar( params, 0, "TimeStepsPerFrame" ) );
            TimeStepsPerSnapshot = unsigned( Mex::Scalar( params, 0, "TimeStepsPerSnapshot" ) );

            CameraAngle      = Mex::Scalar( params, 0, "CameraAngle"     );
            CameraElevation  = Mex::Scalar( params, 0, "CameraElevation" );
            CameraZoom       = Mex::Scalar( params, 0, "CameraZoom"      );

            Mex::Matrix lookAt( params, 0, "CameraLookAt" );
            lookAt.VerifyDims( 1, 3, "CameraLookAt must be a spatial vector" );
            CameraLookAt = WoRB::SpatialVector( lookAt(0), lookAt(1), lookAt(2) );

            FollowObject = unsigned( Mex::Scalar( params, 0, "FollowObject" ) );
            FinalTime    = Mex::Scalar( params, 0, "FinalTime" );

            worb.Collisions.Restitution = Mex::Scalar( params, 0, "Restitution" );
            worb.Collisions.Relaxation  = Mex::Scalar( params, 0, "Relaxation"  );
            worb.Collisions.Friction    = Mex::Scalar( params, 0, "Friction"    );

            Mex::Matrix G( params, 0, "Gravity" );
            G.VerifyDims( 1, 3, "Gravity must be a spatial vector" );
            worb.Gravity = WoRB::SpatialVector( G(0), G(1), G(2) );
        }

        /////////////////////////////////////////////////////////////////////////////////
        // Parse bodies array structure
        //
        if ( nArgIn >= 2 )
        {
            const mxArray* body = argIn[1];
            unsigned bodyCount = unsigned( mxGetNumberOfElements( body ) );

            for ( unsigned i = 0; i < bodyCount; ++i )
            {
                // Get rigid body parameters
                //
                Mex::String geometry( body, i, "Geometry" ); // Geometry class
                Mex::Matrix R( body, i, "HalfExtent" ); // Radius or half-extent
                Mex::Matrix M( body, i, "M" ); // Body mass
                Mex::Matrix X( body, i, "X" ); // Position
                Mex::Matrix Q( body, i, "Q" ); // Oritentation
                Mex::Matrix V( body, i, "V" ); // Velocity
                Mex::Matrix W( body, i, "W" ); // Angular velocity

                Mex::Logical showTrajectory   ( body, i, "ShowTrajectory",   true  );
                Mex::Logical isActive         ( body, i, "CanBeDeactivated", true  );
                Mex::Logical canBeDeactivated ( body, i, "CanBeDeactivated", false );

                Mex::Matrix cActive  ( body, i, "ActiveColor"   ); // Active body color
                Mex::Matrix cInactive( body, i, "InactiveColor" ); // Inactive color

                // Verify dimensions of required rigid body parameters
                //
                M.VerifyDims( 1, 1, "Mass must be a scalar" );
                X.VerifyDims( 1, 3, "Position must be a spatial vector" );
                Q.VerifyDims( 1, 4, "Orientation must be a quaternion" );
                V.VerifyDims( 1, 3, "Velocity must be a spatial vector" );
                W.VerifyDims( 1, 3, "Angular velocity must be a spatial vector" );

                // Create rigid body with the specified geometry
                //
                WoRB::GLUT_Renderer* obj = NULL;

                if ( geometry == "cuboid" )
                {
                    R.VerifyDims( 1, 3,
                        "Half-extent of a cuboid must be a spatial vector" );

                    obj = new WoRB::Box(
                        /* position      */ WoRB::SpatialVector( X(0), X(1), X(2) ), 
                        /* orientation   */ WoRB::Quaternion   ( Q(0), Q(1), Q(2), Q(3) ),
                        /* velocity      */ WoRB::SpatialVector( V(0), V(1), V(2) ), 
                        /* ang. velocity */ WoRB::SpatialVector( W(0), W(1), W(2) ),
                        /* half-extent   */ WoRB::SpatialVector( R(0), R(1), R(2) ), 
                        /* mass          */ M(0)
                    );
                }
                else if ( geometry == "sphere" )
                {
                    R.VerifyDims( 1, 1,
                        "Half-extent i.e. radius of a sphere must be a scalar" );

                    obj = new WoRB::Ball(
                        /* position      */ WoRB::SpatialVector( X(0), X(1), X(2) ), 
                        /* orientation   */ WoRB::Quaternion   ( Q(0), Q(1), Q(2), Q(3) ),
                        /* velocity      */ WoRB::SpatialVector( V(0), V(1), V(2) ), 
                        /* ang. velocity */ WoRB::SpatialVector( W(0), W(1), W(2) ),
                        /* radius        */ R(0), 
                        /* mass          */ M(0)
                    );
                }
                else {
                    WoRB::SevereError( "WoRB:Init:invarg", 
                        "Invalid body(%u) type '%s'; "
                        "allowed types are 'cuboid' or 'sphere'.",
                        i, (const char*)geometry );
                }

                // Show trajectory flag
                //
                obj->ShowTrajectory = showTrajectory;

                // 'Can be deactivated?' and 'is active?' flags
                //
                obj->GetBody().SetCanBeDeactivated( canBeDeactivated );
                if ( ! isActive ) {
                    obj->GetBody().Deactivate ();
                }

                // Rigid body active color, optional
                //
                if ( cActive.IsSize( 1, 4 ) ) {
                    obj->ActiveColor = WoRB::GLUT_Renderer::Colorf( 
                        cActive(0), cActive(1), cActive(2), cActive(3)
                    );
                }

                // Rigid body inactive color, optional
                //
                if ( cInactive.IsSize( 1, 4 ) ) {
                    obj->InactiveColor = WoRB::GLUT_Renderer::Colorf( 
                        cInactive(0), cInactive(1), cInactive(2), cInactive(3)
                    );
                }

                // Finally, add object to the system
                //
                Objects.push_back( obj );
                worb.Add( obj->GetGeometry () );
            }

            // Initialize ODE (calculate derived quantities for recently added objects)
            //
            worb.InitializeODE ();

            // Clear test-suite configuration request since we've configured test-bed.
            //
            TestSuite = -1;
        }

        /////////////////////////////////////////////////////////////////////////////////
        // Report parameters and bodies that we have so far.
        //
        if ( dumpInitialState )
        {
            Dump (); 
            WoRB::Printf( "\n" );
        }
    }

    /** Gets the result matrix.
     */
    mxArray* GetResult ()
    {
        return Result;
    }

    /** Creates the result matrix.
     */
    void CreateResultMatrix ()
    {
        // Ensure to have have a finite number of recorded time-steps
        //
        if ( FinalTime == 0 ) {
            return;  
        }

        unsigned n_steps = unsigned( FinalTime / TimeStep ) + 1;
        Result = Mex::Matrix( n_steps, 11 );

        // Set 'time' column to NaN (indicating row do not have valid data yet).
        //
        for ( unsigned i = 0; i < n_steps; ++i ) {
            Result( i, 0 ) = mxGetNaN ();
        }

        WoRB::Printf( "WoRB: Created matrix for results [ %d × %d ]\n", 
            Result.GetM (), Result.GetN () );
    }

    /** Saves simulated data to be returned to MATLAB as the result.
     */
    virtual void OnProcessData ()
    {
        if ( Result.IsEmpty () ) {
            return;
        }

        unsigned n = worb.TimeStepCount;

        Result( n, 0 ) = worb.Time;
        Result( n, 1 ) = worb.Collisions.Count ();

        Result( n, 2 ) = worb.TotalKineticEnergy;
        Result( n, 3 ) = worb.TotalPotentialEnergy;

        Result( n, 4 ) = worb.TotalLinearMomentum.x;
        Result( n, 5 ) = worb.TotalLinearMomentum.y;
        Result( n, 6 ) = worb.TotalLinearMomentum.z;

        Result( n, 7 ) = worb.TotalAngularMomentum.x;
        Result( n, 8 ) = worb.TotalAngularMomentum.y;
        Result( n, 9 ) = worb.TotalAngularMomentum.z;

        Result( n, 10 ) = Objects.size () == 0 ? 0.0
            : Objects.at( Objects.size() - 1 )->GetBody().Position.y;
    }
};

/////////////////////////////////////////////////////////////////////////////////////////

/** Definition of the static instance wrapper for our GLUT application.
 */
template<class T> 
T* WoRB::GLUT_Framework<T>::Application = 0;

static WoRB_MexFunction Application;
static WoRB::GLUT_Framework<WoRB_MexFunction> glut;

/////////////////////////////////////////////////////////////////////////////////////////

/** The main entry routine for the MATLAB function WoRB.
 */
void mexFunction
(
    int nArgOut, mxArray* argOut[],       // left-hand side (varargout) of the function
    int nArgIn, const mxArray* argIn[]    // right-hand side (varargin) of the function
    )
{
    if ( ! glut.Initialize () ) {
        return;
    }

    // Prevent clearing MEX-file from memory
    //
    if ( ! mexIsLocked () ) {
        mexLock (); 
    }

    // Parse mexFunction input arguments
    //
    Application.Parse( nArgIn, argIn );

    // 
    if ( nArgOut >= 1 ) {
        Application.CreateResultMatrix ();
    }

    Application.SetupAnimation ();

    glut.Connect( Application );

    Application.Run ();

    glut.Disconnect ();

    if ( nArgOut >= 1 ) {
        argOut[0] = Application.GetResult ();
    }
}
