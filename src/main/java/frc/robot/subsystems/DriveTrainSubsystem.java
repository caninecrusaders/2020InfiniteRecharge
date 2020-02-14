/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;

public class DriveTrainSubsystem extends SubsystemBase {
  public static final double WHEELBASE = 18;
  public static final double TRACKWIDTH = 18;

  public boolean enableDrive = true;
  public boolean enableAngle = true;

  public static DriveTrainSubsystem instance;
  public static Mk2SwerveModule mk2SwerveModule;
  public static SwerveModule swerveModule;

  private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
      new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderFL),
              Preferences.getInstance().getDouble("Front Left Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorFL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorFL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();
  private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
      new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderFR),
              Preferences.getInstance().getDouble("Front Right Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorFR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorFR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();
  private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
      new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderBL),
              Preferences.getInstance().getDouble("Back Left Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorBL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorBL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();
  private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
      new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderBR),
              Preferences.getInstance().getDouble("Back Right Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorBR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorBR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
      new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

  private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);
  // private final Gyroscope gyroscope = new NavX(SerialPort.Port.kUSB1);

  public DriveTrainSubsystem() {
    gyroscope.calibrate();
    gyroscope.setInverted(true); // might not need to be inverted

    frontLeftModule.setName("Front Left");
    frontRightModule.setName("Front Right");
    backLeftModule.setName("Back Left");
    backRightModule.setName("Back Right");
  }

  public static DriveTrainSubsystem getInstance() {
    if (instance == null) {
      instance = new DriveTrainSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeftModule.updateSensors();
    frontRightModule.updateSensors();
    backLeftModule.updateSensors();
    backRightModule.updateSensors();

    frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
    frontRightModule.updateState(TimedRobot.kDefaultPeriod);
    backLeftModule.updateState(TimedRobot.kDefaultPeriod);
    backRightModule.updateState(TimedRobot.kDefaultPeriod);

    SmartDashboard.putNumber("Navx", gyroscope.getAngle().toRadians());

  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
    // if (enableAngle && enableDrive) {
      rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
      ChassisSpeeds speeds;
      if (fieldOriented) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
            Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
      } else {
        speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
      }

      SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
      frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
      frontRightModule.setTargetVelocity(-states[1].speedMetersPerSecond, states[1].angle.getRadians());
      backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
      backRightModule.setTargetVelocity(-states[3].speedMetersPerSecond, states[3].angle.getRadians());
    // } else {
      
    //   frontLeftModule.setTargetVelocity(0);
    //   frontRightModule.setTargetVelocity(0);
    //   backLeftModule.setTargetVelocity(0);
    //   backRightModule.setTargetVelocity(0);
    // }

  }

  public void resetGyroscope() {
    gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
  }

  public void saveAllZeroOffsets() {
    double offsetFL = frontLeftModule.getCurrentAngle();
    double offsetFR = frontRightModule.getCurrentAngle();
    double offsetBL = backLeftModule.getCurrentAngle();
    double offsetBR = backRightModule.getCurrentAngle();
    Preferences.getInstance().putDouble("Front Left Offset", offsetFL);
    Preferences.getInstance().putDouble("Front Right Offset", offsetFR);
    Preferences.getInstance().putDouble("Back Left Offset", offsetBL);
    Preferences.getInstance().putDouble("Back Right Offset", offsetBR);

  }


  
}
