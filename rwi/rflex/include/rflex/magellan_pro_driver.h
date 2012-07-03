#ifndef MAGELLAN_PRO_DRIVER_H
#define MAGELLAN_PRO_DRIVER_H

#include <rflex/rflex_driver.h>
#include <sensor_msgs/PointCloud.h>

/**
 * \brief Magellan Pro Driver class
 *  Magellan Pro Driver - By Matthew B. Whitaker 01/2011
 */
class MAGELLAN_PRO : public RFLEX {
    public:
        MAGELLAN_PRO();
        virtual ~MAGELLAN_PRO();
        void setSonarPower(bool);
        float getTransDistance();
        float getRotDistance();
        float getTranslationalVelocity() const;
        float getRotationalVelocity() const;
        float getVoltage() const;
        bool isPluggedIn() const;
        int getNumSonars() const;

        /** Get readings from the sonar in meters
         * \param readings Data structure into which the sonar readings are saved */
        void getSonarReadings(float* readings) const;

        /** Gets a point cloud for sonar readings
         * \param cloud Data structure into which the sonar readings are saved */
        void getSonarPoints(sensor_msgs::PointCloud* cloud) const;

        /** Gets a point cloud for the bump sensors
         * \param cloud Data structure into which the bump readings are saved
         * \return number of active bump sensors
         */
        //int getBumps(sensor_msgs::PointCloud* cloud) const;

        /** Sets the motion of the robot
         * \param tvel Translational velocity (in m/s)
         * \param rvel Rotational velocity (in radian/s)
         * \param acceleration Translational acceleration (in m/s/s) */
        void setMovement(float tvel, float rvel, float acceleration);

        /** Processes the DIO packets - called from RFflex Driver
         * \param address origin
         * \param data values */
        void processDioEvent(unsigned char address, unsigned short data);

        /** Detects whether the robot has all the necessary components
         * to calculate odometry
         * \return bool true if robot has read its distance, bearing and home bearing */
        bool isOdomReady() const {
            return odomReady==3;
        }

    private:
        /*param readings Data structure into which the sonar readings are saved */
        //void getSonarReadings(float* readings) const;
        /*param cloud Data structure into which the sonar readings are saved */
        //void getSonarPoints(sensor_msgs::PointCloud* cloud) const;
        /** \param cloud Data structure into which the bump sensors are saved
            \return number of active bump sensors */
        //int getBumps(sensor_msgs::PointCloud* cloud) const;

        int firstTransDistance;
        int firstRotDistance;

        bool foundTransDistance;
        bool foundRotDistance;

        int** bumps;

        // Not allowed to use these
        MAGELLAN_PRO(const MAGELLAN_PRO &mpro); 			///< Private constructor - Don't use
        MAGELLAN_PRO &operator=(const MAGELLAN_PRO &mpro); 	///< Private constructor - Don't use
};

#endif
