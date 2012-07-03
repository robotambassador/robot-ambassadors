/*
 *  Magellan Pro Driver - By Matthew B. Whitaker 01/2011
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

#include <rflex/magellan_pro_driver.h>
#include <rflex/magellan_pro_config.h>
#include <ros/ros.h>
#include <math.h>

MAGELLAN_PRO::MAGELLAN_PRO() {

	foundTransDistance = false;
    foundRotDistance = false;

    /*bumps = new int[BUMPER_COUNT];

    for (int i=0;i<BUMPER_COUNT;i++) {
            bumps[i] = 0;
    }*/
}

MAGELLAN_PRO::~MAGELLAN_PRO() {
    //delete bumps;
}

float MAGELLAN_PRO::getTransDistance() {

	if (!foundTransDistance && isOdomReady()) {
    	firstTransDistance = transDistance;
    	foundTransDistance = true;
    }

	return (transDistance - firstTransDistance) / (float) ODO_TRANS_DISTANCE_CONVERSION;
}

float MAGELLAN_PRO::getRotDistance() {

	if (!foundRotDistance && isOdomReady()) {
		firstRotDistance = rotDistance;
	    foundRotDistance = true;
	}

	return (rotDistance - firstRotDistance) / (float) ODO_ROT_DISTANCE_CONVERSION;
}

float MAGELLAN_PRO::getTranslationalVelocity() const {
    return transVelocity / (float) ODO_TRANS_DISTANCE_CONVERSION;
}

float MAGELLAN_PRO::getRotationalVelocity() const {
    return rotVelocity / (float) ODO_ROT_DISTANCE_CONVERSION;
}

float MAGELLAN_PRO::getVoltage() const {
    if (voltage==0.0)
        return 0.0;
    else
        return voltage/100.0 + POWER_OFFSET;
}

bool MAGELLAN_PRO::isPluggedIn() const {
    float v = getVoltage();
    if (v>PLUGGED_THRESHOLD)
        return true;
    else
        return false;
}

int MAGELLAN_PRO::getNumSonars() const {
    return NUMBER_SONARS;
}

void MAGELLAN_PRO::setSonarPower(bool on) {
    unsigned long echo, ping, set, val;
    if (on) {
        echo = SONAR_ECHO_DELAY;
        ping = SONAR_PING_DELAY;
        set = SONAR_SET_DELAY;
        val = 2;
    } else {
        echo = ping = set = val = 0;
    }
    configureSonar(echo, ping, set, val);
}


void MAGELLAN_PRO::setMovement( float tvel, float rvel,
                       float acceleration ) {
    setVelocity(tvel * ODO_TRANS_DISTANCE_CONVERSION,
                rvel * ODO_ROT_DISTANCE_CONVERSION,
                acceleration * ODO_TRANS_DISTANCE_CONVERSION);
}


void MAGELLAN_PRO::getSonarReadings(float* adjusted_ranges) const {
    int i = 0;
    for (int x = 0; x < SONAR_NUM_BANKS; x++) {
        for (int y = 0; y < SONAR_PER_BANK; y++) {
            int range = sonar_ranges[( x * SONAR_PER_BANK ) + y];
            if (range > SONAR_MAX_RANGE)
                range = SONAR_MAX_RANGE;
            float fRange = range / (float) RANGE_CONVERSION;
            adjusted_ranges[i] = fRange;
            i++;
        }
    }
}

void MAGELLAN_PRO::getSonarPoints(sensor_msgs::PointCloud* cloud) const {

	float* readings = new float[NUMBER_SONARS];
    getSonarReadings(readings);
    cloud->set_points_size(NUMBER_SONARS);
    int c = 0;
    for (int i = 0; i < NUMBER_SONARS; ++i) {
        if (readings[i] < SONAR_MAX_RANGE/ (float) RANGE_CONVERSION) {
            double angle =  SONAR_RING_START_ANGLE + SONAR_RING_ANGLE_INC*i;
            angle *= M_PI / 180.0;

            double d = SONAR_RING_DIAMETER + readings[i];

            if (i == 0) {
            	printf("Reading 0 = %d", readings[0]);
            }

            cloud->points[c].x = cos(angle)*d;
            cloud->points[c].y = sin(angle)*d;
            cloud->points[c].z = SONAR_RING_HEIGHT;
            c++;
        }
    }
}

/*int MAGELLAN_PRO::getBumps(sensor_msgs::PointCloud* cloud) const {
    int c = 0;
    double wedge = 2 * M_PI / BUMPER_COUNT;
    double d = SONAR_RING_DIAMETER*1.1;
    int total = 0;
    for (int i=0;i<BUMPER_COUNT;i++) {
        int value = bumps[i];
        for (int j=0;j<4;j++) {
            int mask = 1 << j;
            if ((value & mask) > 0) {
                total++;
            }
        }
    }

    cloud->set_points_size(total);
    if (total==0)
        return 0;
    for (int i=0;i<BUMPER_COUNT;i++) {
        int value = bumps[i];
        double angle = wedge * (2.5 - i);
        for (int j=0;j<4;j++) {
            int mask = 1 << j;
            if ((value & mask) > 0) {
                double aoff = BUMPER_ANGLE_OFFSET[j]*wedge/3;
                cloud->points[c].x = cos(angle-aoff)*d;
                cloud->points[c].y = sin(angle-aoff)*d;
                cloud->points[c].z = BUMPER_HEIGHT_OFFSET[index][j];
                c++;
            }
        }
    }
    return total;

}*/

void MAGELLAN_PRO::processDioEvent(unsigned char address, unsigned short data) {

	ROS_INFO("MAGELLAN_PRO DIO: address 0x%02x", address);

    if (address == HEADING_HOME_ADDRESS) {
        //home_bearing = bearing;
        //printf("Godot Home %f \n", home_bearing / (float) ODO_ANGLE_CONVERSION);
    }// check if the dio packet came from a bumper packet
    /*else if ((address >= BUMPER_ADDRESS) && (address < (BUMPER_ADDRESS+BUMPER_COUNT))) {
        int index =0, rot = address - BUMPER_ADDRESS;
        if (rot > BUMPERS_PER[index]) {
            rot -= BUMPERS_PER[index];
            index++;
        }
        bumps[index][rot] = data;
    }*/ else {
        printf("MAGELLAN_PRO DIO: address 0x%02x (%d) value 0x%02x (%d)\n", address, address, data, data);
    }
}
