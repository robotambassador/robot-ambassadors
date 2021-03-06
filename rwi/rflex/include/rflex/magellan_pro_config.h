/*  Magellan Pro constants
 *  Matthew B. Whitaker - 01/2011
 *  Modified from ROS B21 source
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef MAGELLAN_PRO_CONFIG_H
#define MAGELLAN_PRO_CONFIG_H

// Odometery Constants
// ===================
// Arbitrary units per meter
// Left is -ive and Right is +ive if going "forwards"
#define ODO_TRANS_DISTANCE_CONVERSION 39800
#define ODO_ROT_DISTANCE_CONVERSION 35000

// Sonar Constants
// ===============
// Arbitrary units per meter (for sonar)
#define RANGE_CONVERSION 1476
#define SONAR_ECHO_DELAY 30000
#define SONAR_PING_DELAY 0
#define SONAR_SET_DELAY  0
#define SONAR_MAX_RANGE 3000

#define SONAR_PER_BANK 8
#define SONAR_NUM_BANKS 2

const int NUMBER_SONARS = 16;

const float SONAR_RING_START_ANGLE = 180;
const float SONAR_RING_ANGLE_INC = -15;
const float SONAR_RING_DIAMETER = .25;
const float SONAR_RING_HEIGHT = 0.055;

// Digital IO constants
// ====================
#define HEADING_HOME_ADDRESS 0x31
#define HOME_BEARING -32500

#define BUMPER_ADDRESS 0x40
#define BUMPER_COUNT 16
#define BUMPER_ADDRESS_STYLE 0
#define BUMPER_BIT_STYLE 1
#define BUMPER_STYLE 0
const int BUMPERS_PER = 8;
const double BUMPER_ANGLE_OFFSET[] = {-1,1,-1,1};
const double BUMPER_HEIGHT_OFFSET[][4] = {{.5,.5,.05,.05},
    {.25,.25,.05,.05}
};

// IR Constants
// ============
#define IR_POSES_COUNT 16
#define IR_BASE_BANK 0
#define IR_BANK_COUNT 2
#define IR_PER_BANK 8

// Misc Constants
// ==============
#define USE_JOYSTICK 0
#define JOY_POS_RATIO 6
#define JOY_ANG_RATIO -0.01
#define POWER_OFFSET 1.2
#define PLUGGED_THRESHOLD 25.0

#endif
