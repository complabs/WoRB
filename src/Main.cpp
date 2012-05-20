/**
 *  @file      Main.cpp
 *  @brief     The main entry point for both the stand-alone application.
 *  @author    Mikica Kocic
 *  @version   0.3
 *  @date      2012-05-10
 *  @copyright GNU Public License.
 */

#include "WoRB.h"
#include "Utilities.h"
#include "WoRB_TestBed.h"

/////////////////////////////////////////////////////////////////////////////////////////

/** Definition of the static instance wrapper for our GLUT application.
 */
template<class T> 
T* WoRB::GLUT_Framework<T>::Application = 0;

static WoRB_TestBed Application;
static WoRB::GLUT_Framework<WoRB_TestBed> glut;

/////////////////////////////////////////////////////////////////////////////////////////

/** The main entry point for the stand-alone version.
 */
int main( int argc, char* argv [] )
{
    if ( ! glut.Initialize( &argc, argv ) ) {
        return 0;
    }

    Application.Initialize ();
    Application.SetupAnimation ();

    glut.Connect( Application );

    Application.Run ();

    glut.Terminate ();

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
