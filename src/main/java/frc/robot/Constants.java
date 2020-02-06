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
    public static final int leftMotor = 2;
    public static final int rightMotor = 3;

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
    
    public static final int climbExtendMotorID = 10;
    public static final int climbWinchMotorID = 11;
    public static final int collectorMotorID = 12;
    public static final int shootRightMotorID = 13;
    public static final int shootLeftMotorID = 14;

    public static final int actuatorModuleID = 1;
    public static final int actuatorRightPistonExtendID = 0;
    public static final int actuatorRightPistonRetractID = 1;
    public static final int actuatorCollectorExtendID = 2;
    public static final int actuatorCollectorRetractID = 3;
    public static final int actuatorClimbExtendID = 4;
    public static final int actuatorClimbRetractID = 5;
    public static final int actuatorLeftPistonExtendID = 6;
    public static final int actuatorLeftPistonRetractID = 7;
}
