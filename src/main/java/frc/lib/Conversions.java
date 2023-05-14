// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

/** Add your docs here. */
public class Conversions {
    
    /**
     * Converts from Falcon position counts to degrees
     * @param counts Revolutions of motor in Falcon units
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Position of output in degrees
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * converts from degrees to Falcon position counts
     * @param degrees Revolutions of output in # of revolutions
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Position of motor in Falcon units
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * Converts from Falcon velocity counts to revolutions per minute
     * @param velocityCounts Velocity of Falcon in counts per 100ms
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Velocity of output in revolutions per minute
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * Converts from Falcon position counts to revolutions
     * @param positionCounts Revolutions of motor in Falcon units
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Position of output in # of revolutions
     */
    public static double falconToRevolutions(double positionCounts, double gearRatio) {
        double motorRevolutions = positionCounts / 2048.0;        
        double mechRevolutions = motorRevolutions / gearRatio;
        return mechRevolutions;
    }

    /**
     * Converts from revolutions per minute to Falcon velocity counts
     * @param RPM Velocity of output in revolutions per minute
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Velocity of Falcon in Falcon counts per 100ms
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * Converts from Falcon velocity counts to meters per second
     * @param velocityCounts Velocity of Falcon in counts per 100ms
     * @param circumference Circumference of wheel or pitch diameter on mechanism
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Velocity of output in meters per second
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * Converts from Falcon position counts to meters
     * @param positionCounts Revolutions of motor in Falcon units
     * @param circumference Circumference of wheel or pitch diameter on mechanism
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Position output in meters
     */
    public static double falconToMeters(double positioncounts, double circumference, double gearRatio){
        double wheelRevolutions = falconToRevolutions(positioncounts, gearRatio);
        double wheelMeters = (wheelRevolutions * circumference);
        return wheelMeters;
    }

    /**
     * Converts from meters per second to Falcon velocity counts
     * @param velocity Velocity of output in meters per second
     * @param circumference Circumference of wheel or pitch diameter on mechanism
     * @param gearRatio Gear ratio between falcon an mechanism
     * @return Velocity of Falcon in Falcon counts per 100ms
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
}
