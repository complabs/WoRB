#ifndef _WORB_H_INCLUDED
#define _WORB_H_INCLUDED

/**
 *  @file      WoRB.h
 *  @brief     The World of Rigid Bodies (WoRB) Simulation Framework.
 *  @author    Mikica B Kocic
 *  @version   0.8
 *  @date      2012-05-05
 *  @copyright GNU Public License.
 */

#include "RigidBody.h"
#include "CollisionResolver.h"

namespace WoRB
{
    /** Encapsulates a system of rigid bodies.
     */
    template  
    <
        /** The maximum number of rigid bodies the system can have.
         */
        unsigned MaxObjects,

        /** The maximum number of collisions the system can register.
         */
        unsigned MaxCollisions
    >
    class WorldOfRigidBodies
    {
        /** Represents a rigid body iterator for the WorldOfRigidBodies class.
         * Iterates only through rigid bodies avoiding scenery geometries.
         */
        class RigidBodies
        {
            WorldOfRigidBodies* worb; //!< Holds an instance where we iterate 
            unsigned i;               //!< The current geometry

        public:

            /** Constructs rigid body iterator for the given instance of WoRB.
             */
            RigidBodies( WorldOfRigidBodies* worb ) 
                : worb(worb), i(0)
            {
                while( i < worb->ObjectCount && worb->Object[i]->Body == 0 ) {
                    ++i;
                }
            }

            /** Advances to the next rigid body
             */
            RigidBodies& operator++ ()
            {
                i = Next( i + 1 );
                return *this;
            }

            /** Returns true if the current geometry is connected to a rigid body
             */
            bool Exists ()
            {
                return i < worb->ObjectCount && worb->Object[i]->Body != 0;
            }

            /** Locates next rigid body from the given location
             */
            unsigned Next( unsigned index )
            {
                while( index < worb->ObjectCount && worb->Object[index]->Body == 0 ) {
                    ++index;
                }
                return index;
            }

            /** Gets the pointer to the current rigid body.
             */
            RigidBody* operator -> () 
            {
                return i < worb->ObjectCount ? worb->Object[i]->Body : 0;
            }

            /** Removes all objects from the WoRB instance.
             */
            void Clear ()
            {
                worb->ObjectCount = 0;
                i = 0;
            }

            /** Removes all objects from the WoRB instance.
             */
            unsigned Count ()
            {
                return worb->ObjectCount;
            }
        };

        /////////////////////////////////////////////////////////////////////////////////

        friend class RigidBodies;

        /** Holds the number of the objects in the system.
         */
        unsigned ObjectCount;

        /** Points to an array of the objects handled by the system.
         */
        Geometry* Object[ MaxObjects ];

    public:

        /////////////////////////////////////////////////////////////////////////////////

        /** Holds common gravity applied to all objects (set to 0 to disable).
         */
        Quaternion Gravity;

        /////////////////////////////////////////////////////////////////////////////////

        /** Holds the system local time, in `s`.
         */
        double Time;

        /** Holds the number of integrator time-steps, from the simulation start.
         */
        unsigned long TimeStepCount;

        /** Holds the total kinetic energy of the system, in `J`.
         */
        double TotalKineticEnergy;

        /** Holds the total potential energy of the system, in `J`.
         */
        double TotalPotentialEnergy;

        /** Holds the total linear momentum of the system, in `kg m^2 s^-1`.
         */
        Quaternion TotalLinearMomentum;

        /** Holds the total angular momentum of the system, in `kg m^2 s^-1`.
         */
        Quaternion TotalAngularMomentum;

        /////////////////////////////////////////////////////////////////////////////////

        /** Holds the collision data for all registered collisions.
         */
        CollisionResolver Collisions;

        /** Holds the memory parcel where collisions are registered.
         */
        Collision CollisionRegistry[ MaxCollisions ];

        /////////////////////////////////////////////////////////////////////////////////

        /** Constructs an instance of WoRB class.
         */
        WorldOfRigidBodies ()
            : Collisions( CollisionRegistry, MaxCollisions )
        {
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Removes all objects from the system.
         */
        void RemoveObjects ()
        {
            ObjectCount = 0;
        }

        /** Adds new object to the system.
         */
        void Add( Geometry* object )
        {
            Object[ ObjectCount++ ] = object;
        }

        /** Adds new object to the system.
         */
        void Add( Geometry& object )
        {
            Object[ ObjectCount++ ] = &object;
        }

        /////////////////////////////////////////////////////////////////////////////////

        /** Prepares ODE (recalculates derived quantities)
         */
        void InitializeODE ()
        {
            Time          = 0;
            TimeStepCount = 0;

            Collisions.Initialize ();

            for ( RigidBodies body = RigidBodies(this); body.Exists(); ++body )
            {
                body->CalculateDerivedQuantities ();
                body->ClearAccumulators ();
            }

            TotalKineticEnergy   = 0;
            TotalPotentialEnergy = 0;
            TotalLinearMomentum  = 0;
            TotalAngularMomentum = 0;

            for ( RigidBodies body = RigidBodies(this); body.Exists(); ++body )
            {
                TotalKineticEnergy   += body->KineticEnergy;
                TotalPotentialEnergy += body->PotentialEnergy;
                TotalLinearMomentum  += body->LinearMomentum;
                TotalAngularMomentum += body->TotalAngularMomentum;
            }
        }

        /** Solves (integrates) equations of motion for the whole system
         *
         * Solves ODE for all the objects in the system, performs the contact generation
         * and then resolves detected contacts.
         */
        void SolveODE(
            double h    //!< Integrator time-step length
            )
        {
            /////////////////////////////////////////////////////////////////////////////
            // Calculate and accumulate all external and internal forces
            //

            // Add gravity
            //
            for ( RigidBodies body = RigidBodies(this); body.Exists(); ++body )
            {
                Quaternion f_g = body->Mass() * Gravity;
                double E_p = - f_g.Dot( body->Position );

                body->AddExternalForce( f_g, E_p );
            }

            /////////////////////////////////////////////////////////////////////////////
            // Solve ODE for every object in the system
            //
            for ( RigidBodies body = RigidBodies(this); body.Exists(); ++body )
            {
                body->SolveODE( h );
            }

            // Solve system local time (avoiding `Time += h` cause of rounding-errors).
            //
            Time = h * (++TimeStepCount);

            /////////////////////////////////////////////////////////////////////////////
            // Calculate derived quantities

            TotalKineticEnergy   = 0;
            TotalPotentialEnergy = 0;
            TotalLinearMomentum  = 0;
            TotalAngularMomentum = 0;

            for ( RigidBodies body = RigidBodies(this); body.Exists(); ++body )
            {
                TotalKineticEnergy   += body->KineticEnergy;
                TotalPotentialEnergy += body->PotentialEnergy;
                TotalLinearMomentum  += body->LinearMomentum;
                TotalAngularMomentum += body->TotalAngularMomentum;
            }

            /////////////////////////////////////////////////////////////////////////////
            // Collision Detection

            Collisions.Initialize ();

            // Detect and register collisions between all objects in the system
            //
            for ( unsigned i = 0; i < ObjectCount; ++i )
            {
                for ( unsigned j = i + 1; j < ObjectCount; ++j )
                {
                    Object[i]->Detect( Collisions, Object[j] );
                }
            }

            Collisions.UpdateDerivedQuantities( h );

            /////////////////////////////////////////////////////////////////////////////
            // Collision Response

            Collisions.ImpulseTransfers( h );
            Collisions.PositionProjections ();

            /////////////////////////////////////////////////////////////////////////////
            // Prepare force and torque accumulators for the next time-step
            //
            for ( RigidBodies body = RigidBodies(this); body.Exists(); ++body )
            {
                body->ClearAccumulators ();
            }
        }
    };

    /** User defined ANSI C printf-style output routine.
     */
    void Printf( const char* format, ... );

    /** Reports a severe error (with errorId compatible with MATLAB) and quits.
     */
    void SevereError( const char* errorId, const char* format, ... );
}

/**
 * @mainpage World of (Rigid) Bodies Simulation Framework
 *
 * World of Bodies (WoRB) is a system for real-time simulation of rigid bodies. 
 *
 * @section use How to use WoRB framework
 *
 * Initialization
 *
 * @li Create a set of instances of Geometry and RigidBody
 * @li Set mass and inertia tensor for the rigid bodies
 * @li Set their initial location, orientation, velocity and angular velocity
 * @li Apply any permanent external forces, e.g. gravity
 *
 * @li Create an instance of WorldOfRigidBodies
 * @li Populate WorldOfRigidBodies::Objects with the list of instantiated rigid bodies
 *
 * The main loop of the simulation
 *
 * @li Apply any temporary internal or external forces, e.g. spring forces or thrusts.
 * @li Call WorldOfRigidBodies::SolveODE
 * @li Render the bodies.
 *
 * The single step of the simulation is in WorldOfRigidBodies::SolveODE, which
 * performs the following steps
 *
 * @li Solve ODE on each body in the system
 * @li Collision detection: find all collisions between all geometries
 * @li Collision response: resolve all the collisions
 * @li Update the internal derived quantities of the rigid bodies and geometries
 */

#endif // _WORB_H_INCLUDED
