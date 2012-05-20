#ifndef _CONSTANTS_H_INCLUDED
#define _CONSTANTS_H_INCLUDED

/**
 *  @file      Constants.h
 *  @brief     Definitions for the Const static class that encapsulates various
 *             physical and mathematical constants.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-05-06
 *  @copyright GNU Public License.
 */

namespace WoRB 
{
    class Quaternion;

    /////////////////////////////////////////////////////////////////////////////////////
    /** @class Const
     *
     * Physical and mathematical constants.
     */
    class Const
    {
    public:

        /////////////////////////////////////////////////////////////////////////////////

        /** Represents X-axis unit vector.
         */
        const static Quaternion X;

        /** Represents Y-axis unit vector.
         */
        const static Quaternion Y;

        /** Represents Z-axis unit vector.
         */
        const static Quaternion Z;

        /////////////////////////////////////////////////////////////////////////////////

        /** Gets standard gravity (standard acceleration due to free fall in kg m/s^2).
         * @note Standard gravity is given along Y-axis as vertical axis.
         */
        const static Quaternion g_n;

        /////////////////////////////////////////////////////////////////////////////////

        /** Represents Pi.
         */
        const static double Pi;

        /////////////////////////////////////////////////////////////////////////////////

        /** Represents the maximum finite value for a double floating-point number.
         */
        const static double Max;

        /** Represents the minimum finite value for a double floating-point number.
         */
        const static double Min;

        /** Gets the smallest positive `x` such that `x + Eps + x` is representable.
         */
        const static double Eps;    

        /** Returns the representation of positive infinity.
         */
        const static double Inf;

        /** Returns the representation of a quiet not a number (NaN).
         */
        const static double NaN;

        /** Returns true if the argument is NaN.
         */
        static bool IsNaN( double x )
        {
            return x != x;
        }

        /** Returns +1/-1 if the argument is positive/negative infinity.
         */
        static int IsInf( double x )
        {
            double delta = x - x;
            if ( x == x && delta != 0.0 ) {
                return x < 0.0 ? -1 : 1;
            }
            return 0;
        }

        /////////////////////////////////////////////////////////////////////////////////
    };

} // namespace WoRB

#endif // _QUATERNION_H_INCLUDED
