#ifndef _WORB_COLLISION_RESOLVER_H_INCLUDED
#define _WORB_COLLISION_RESOLVER_H_INCLUDED

/**
 *  @file      CollisionResolver.h
 *  @brief     Definitions for the CollisionResolver class which implements
 *             collision response algorithms for the system of rigid bodies.
 *  @author    Mikica Kocic
 *  @version   0.16
 *  @date      2012-05-02
 *  @copyright GNU Public License.
 */

#include "Geometry.h"
#include "Collision.h"

namespace WoRB 
{
    /** Encapsulates collision response framework.
     *
     * All collisions in the system, after detection, are registered in an
     * instance of the CollisionResolver class and then resolved using 
     * the impulse transfer and the position projections methods.
     *
     * The CollisionResolver instance may be shared between different WoRB systems.
     */
    class CollisionResolver
    {
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Private properties                                                   */
                                                                                   /*@{*/
        /** Holds the maximum number of collisions the array can take.
         */
        unsigned MaxCollisionCount;

        /** Holds the parcel for the collision data.
         * Allocation of the parcel is left to the user (may be dynamic or static).
         */
        Collision* Collisions;

        /** Holds the next free place in the parcel.
         */
        Collision* NextFree;

        /** Holds the free space in allocation area.
         */
        unsigned FreeCount;

        /** Holds the current number of collisions found.
         */
        unsigned CollisionCount;

    public:
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Public properties                                                    */
                                                                                   /*@{*/
        /** Holds the restitution coefficient common for all collisions.
         */
        double Restitution;

        /** Holds the position projections coefficient common for all collisions.
         */
        double Relaxation;

        /** Holds the friction coefficient common for all collisions.
         */
        double Friction;
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Constructor                                                          */
                                                                                   /*@{*/
        /** Instantiates a collision detection framework in the given area.
         */
        CollisionResolver( Collision* allocationArea, unsigned length )
            : MaxCollisionCount( length )
            , Collisions( allocationArea )
            , NextFree( allocationArea )
            , FreeCount( length )
            , CollisionCount( 0 )
            , Restitution( 1.0 )
            , Relaxation( 0.2 )
            , Friction( 0.0 )
        {
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Property getters and index operations                                */
                                                                                   /*@{*/
        /** Gets number of contats in the registry.
         */
        unsigned Count () const
        {
            return CollisionCount;
        }

        /** Checks if there is a free space available for more contact data.
         */
        bool HasSpaceForMoreContacts () const
        {
            return FreeCount > 0;
        }

        /** Gets collision data with the specified index.
         */
        const Collision& operator [] ( unsigned index ) const
        {
            return Collisions[ index ];
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Collision detection methods                                          */
                                                                                   /*@{*/
        /** Clears the collision registry.
         */
        void Initialize ()
        {
            NextFree = Collisions;
            FreeCount = MaxCollisionCount;
            CollisionCount = 0;
        }

        /** Registers a new contact.
         * @return 1 if ok, 0 if failed to allocate space for the contact.
         */
        unsigned RegisterNewContact
        (
            RigidBody* body_A,           //!< The first body
            RigidBody* body_B,           //!< The second body; 0 in case of scenery
            const Quaternion& position,  //!< The position of the contact in world frame
            const Quaternion& normal,    //!< The contact normal in world frame
            double penetration           //!< The penetration depth
            )
        {
            if ( FreeCount <= 0 ) {
                return 0;
            }

            // Add contact the list of maintained collisions.
            //
            NextFree->Body_A        = body_A;
            NextFree->Body_B        = body_B;
            NextFree->Position      = position;
            NextFree->Normal        = normal;
            NextFree->Penetration   = penetration;
            NextFree->Friction      = Friction;
            NextFree->Restitution   = Restitution;

            // Reduce the number of remainging contacts and advance the array forward.
            //
            ++NextFree; --FreeCount;
            ++CollisionCount;

            return 1;
        }

        /** Updates drived quantities (like contact velocity and axis info).
         */
        void UpdateDerivedQuantities( double timeStep )
        {
            for ( unsigned i = 0; i < CollisionCount; ++i )
            {
                Collisions[i].UpdateDerivedQuantities( timeStep );
            }
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Collision response methods                                           */
                                                                                   /*@{*/
        /** Resolves collisions using the impulse transfer method.
         */
        void ImpulseTransfers( double timeStep, 
            unsigned maxIterations = 0, double velocityEPS = 0.01 );

        /** Resolves collisions using the position projection method.
         */
        void PositionProjections(
            unsigned maxIterations = 0, double positionEPS = 0.01 );
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
        /** @name Miscellanous methods                                                 */
                                                                                   /*@{*/
        /** Displays information about all registered collisions on standard output.
         */
        public: void Dump( double currentTime ) const;

        /** Find a collision with the maximum bouncing velocity.
         */
        Collision* FindLargestBouncingVelocity( double eps )
        {
            Collision* contact = 0;
            for ( unsigned i = 0; i < CollisionCount; i++ )
            {
                if ( Collisions[i].BouncingVelocity > eps )
                {
                    eps = Collisions[i].BouncingVelocity;
                    contact = &Collisions[i];
                }
            }
            return contact;
        }

        /** Finds a collision with the largest penetration.
         */
        Collision* FindLargestPenetration( double eps )
        {
            Collision* contact = 0;
            for ( unsigned i = 0; i < CollisionCount; ++i )
            {
                if ( Collisions[i].Penetration > eps )
                {
                    eps = Collisions[i].Penetration;
                    contact = &Collisions[i];
                }
            }
            return contact;
        }
                                                                                   /*@}*/
        /////////////////////////////////////////////////////////////////////////////////
    };

} // namespace WoRB

#endif // _WORB_COLLISION_RESOLVER_H_INCLUDED
