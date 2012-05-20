/**
 *  @file      Constants.cpp
 *  @brief     Implementation of the Const class that encapsulates various
 *             physical and mathematical constants.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-05-03
 *  @copyright GNU Public License.
 */

#include "Constants.h"
#include "Quaternion.h"

#include <cmath>  // we use atan
#include <limits> // we use std::numeric_limits
#include <cstdio> // we use: printf

using namespace WoRB;

/////////////////////////////////////////////////////////////////////////////////////////

const double Const::Pi  = 4 * std::atan(1.0);

/////////////////////////////////////////////////////////////////////////////////////////

const double Const::Max = std::numeric_limits<double>::max ();
const double Const::Min = std::numeric_limits<double>::min ();
const double Const::Eps = std::numeric_limits<double>::epsilon ();
const double Const::Inf = std::numeric_limits<double>::infinity ();
const double Const::NaN = std::numeric_limits<double>::quiet_NaN ();

/////////////////////////////////////////////////////////////////////////////////////////

const Quaternion Const::g_n( 0, 0, -9.80665, 0 ); // Standard gravity along y-axis

/////////////////////////////////////////////////////////////////////////////////////////

const Quaternion Const::X ( 0, 1, 0, 0 );
const Quaternion Const::Y ( 0, 0, 1, 0 );
const Quaternion Const::Z ( 0, 0, 0, 1 );

