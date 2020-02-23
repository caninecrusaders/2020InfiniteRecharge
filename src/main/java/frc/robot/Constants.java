/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // public static final int leftMotor = 2;
    // public static final int rightMotor = 3;
    public static final int blinkinController = 0; 

    public static final int encoderFL = 1;
    public static final int encoderBL = 2;
    public static final int encoderFR = 0;
    public static final int encoderBR = 3;

    public static final int driveMotorFL = 2;
    public static final int angleMotorFL = 3;
    public static final int driveMotorBL = 4;
    public static final int angleMotorBL = 5;
    public static final int driveMotorFR = 6;
    public static final int angleMotorFR = 7;
    public static final int driveMotorBR = 8;
    public static final int angleMotorBR = 9;
    
    public static final int collectorMotorID = 10;
    public static final int beltMotorID = 11;
    public static final int climbExtendMotorID = 12;
    public static final int climbWinchMotorOneID = 13;
    public static final int climbWinchMotorTwoID = 14;
    public static final int shooterMotorID = 15;

    public static final int actuatorModuleID = 1;
    public static final int actuatorPistonExtendID = 0;
    public static final int actuatorPistonRetractID = 1;
    public static final int actuatorCollectorExtendID = 2;
    public static final int actuatorCollectorRetractID = 3;
    public static final int actuatorClimbExtendID = 4;
    public static final int actuatorClimbRetractID = 5;
    
    public static final int detectFuelTriggerID = 8;
    public static final int detectFuelEchoID = 9;
    //for digital input
    public static final int digitalSensorIntakeID = 7;
    public static final int digitalSensorGateID = 6;

    //public static final int sensorID = 4;

    public static final double LowShooterRunTime = 1.0;

}
