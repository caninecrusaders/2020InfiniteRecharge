/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;

import java.util.Optional;
import java.util.Vector;

import javax.annotation.concurrent.GuardedBy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModule;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.NavX.Axis;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;

public class DriveTrainSubsystem extends SubsystemBase implements UpdateManager.Updatable {
  public static final double WHEELBASE = 18;
  public static final double TRACKWIDTH = 18;
  private final double wheelDiameter = 4.1;
  private boolean fullSpeedTurn = false;

  private PIDController snapPID = new PIDController(0.2, 0.01, 0.0);
  public enum RotationMode {kManual, kZero, kBar};
  private RotationMode rotationMode = RotationMode.kManual;
  public ChassisVelocity velocity;

  private double startRotations[] = {0,0,0,0};
  public static DriveTrainSubsystem instance;
  
  private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
  private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
  private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
  private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
      new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0));

  public PidController snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
  private double snapRotation = Double.NaN;

  
  private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
      FOLLOWER_TRANSLATION_CONSTANTS, FOLLOWER_ROTATION_CONSTANTS, FOLLOWER_FEEDFORWARD_CONSTANTS);

  private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
      new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderFL),
              Preferences.getInstance().getDouble("Front Left Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorFL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorFL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();
  private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
      new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderFR),
              Preferences.getInstance().getDouble("Front Right Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorFR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorFR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();
  private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
      new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderBL),
              Preferences.getInstance().getDouble("Back Left Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorBL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorBL, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();
  private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
      new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
          .angleEncoder(new AnalogInput(Constants.encoderBR),
              Preferences.getInstance().getDouble("Back Right Offset", 0))
          .angleMotor(new CANSparkMax(Constants.angleMotorBR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .driveMotor(new CANSparkMax(Constants.driveMotorBR, MotorType.kBrushless),
              Mk2SwerveModuleBuilder.MotorType.NEO)
          .build();

  private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

  private final SwerveKinematics kinematics = new SwerveKinematics(new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // fl
      new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // fr
      new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // bl
      new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)); // br

  // public final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);
  // public final Gyroscope gyroscope = new NavX(SerialPort.Port.kUSB1);
  private final SwerveOdometry odometry = new SwerveOdometry(kinematics, RigidTransform2.ZERO);

  private final Object sensorLock = new Object();
  @GuardedBy("sensorLock")
  public final NavX navX = new NavX(SPI.Port.kMXP);

  private final Object kinematicsLock = new Object();
  @GuardedBy("kinematicsLock")
  private RigidTransform2 pose = RigidTransform2.ZERO;

  private final Object stateLock = new Object();
  @GuardedBy("stateLock")
  private HolonomicDriveSignal driveSignal = null;

  // Logging stuff
  private NetworkTableEntry poseXEntry;
  private NetworkTableEntry poseYEntry;
  private NetworkTableEntry poseAngleEntry;

  private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

  public DriveTrainSubsystem() {
    synchronized (sensorLock) {
      navX.setInverted(true);
      
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    poseXEntry = tab.add("Pose X", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
    poseYEntry = tab.add("Pose Y", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
    poseAngleEntry = tab.add("Pose Angle", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();

    ShuffleboardLayout frontLeftModuleContainer = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withPosition(1, 0).withSize(2, 3);
    moduleAngleEntries[0] = frontLeftModuleContainer.add("Angle", 0.0).getEntry();

    ShuffleboardLayout frontRightModuleContainer = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withPosition(3, 0).withSize(2, 3);
    moduleAngleEntries[1] = frontRightModuleContainer.add("Angle", 0.0).getEntry();

    ShuffleboardLayout backLeftModuleContainer = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
        .withPosition(5, 0).withSize(2, 3);
    moduleAngleEntries[2] = backLeftModuleContainer.add("Angle", 0.0).getEntry();

    ShuffleboardLayout backRightModuleContainer = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        .withPosition(7, 0).withSize(2, 3);
    moduleAngleEntries[3] = backRightModuleContainer.add("Angle", 0.0).getEntry();


    snapPID.enableContinuousInput(-180.0, 180.0);
    snapPID.setIntegratorRange(-0.5, 0.5);
    snapPID.setTolerance(0.5); 
    snapPID.reset();
  }
  
  public RigidTransform2 getPose() {
    synchronized (kinematicsLock) {
      return pose;
    }
  }

  public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
    synchronized (stateLock) {
      driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, fieldOriented);
    }
  }

  public void drive(HolonomicDriveSignal newSignal) {
    synchronized (stateLock) {
      driveSignal = newSignal;
    }
  }

  public void resetGyroAngle(Rotation2 angle) {
    synchronized (sensorLock) {
      navX.setAdjustmentAngle(navX.getUnadjustedAngle().rotateBy(angle.inverse()));
    }
  }

  public void update(double timestamp, double dt) {
    updateOdometry(dt);

    HolonomicDriveSignal driveSignal;
    synchronized (stateLock) {
      driveSignal = this.driveSignal;
    }

    updateModules(driveSignal, dt);
  }

  private void updateOdometry(double dt) {
    Vector2[] moduleVelocities = new Vector2[modules.length];
    for (int i = 0; i < modules.length; i++) {
      var module = modules[i];
      module.updateSensors();

      moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle()))
          .scale(module.getCurrentVelocity());
    }

    Rotation2 angle;
    synchronized (sensorLock) {
      angle = navX.getAngle();
    }

    RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

    synchronized (kinematicsLock) {
      this.pose = pose;
    }
  }

  private void updateModules(HolonomicDriveSignal signal, double dt) {
    if (signal == null) {
      velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
    } else if (signal.isFieldOriented()) {
      velocity = new ChassisVelocity(signal.getTranslation().rotateBy(getPose().rotation.inverse()),
          signal.getRotation());
    } else {
      velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
    }

    Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
    SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

    for (int i = 0; i < modules.length; i++) {
      var module = modules[i];
      module.setTargetVelocity(moduleOutputs[i]);
      module.updateState(dt);
    }
  }

  @Override
  public void periodic() {
    // update(timestamp, dt);
    var pose = getPose();
    poseXEntry.setDouble(pose.translation.x);
    poseYEntry.setDouble(pose.translation.y);
    poseAngleEntry.setDouble(pose.rotation.toDegrees());
    SmartDashboard.putNumber("Yaw", navX.getAxis(Axis.YAW));

    for (int i = 0; i < modules.length; i++) {
      var module = modules[i];
      moduleAngleEntries[i].setDouble(Math.toDegrees(module.getCurrentAngle()));
    }
  }

  public static DriveTrainSubsystem getInstance() {
    if (instance == null) {
      instance = new DriveTrainSubsystem();
    }
    return instance;
  }

  public HolonomicMotionProfiledTrajectoryFollower getFollower() {
    return follower;
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

  public void enableFullSpeedTurn(boolean flag) {
    fullSpeedTurn = flag;
  }

  public boolean isFullSpeedTurn() {
    return fullSpeedTurn;
  }
  public void setSnapRotation(double targetAngle) {
    snapPID.reset();
    snapPID.setSetpoint(targetAngle);
  }

  public double snapRotation() {
    // calculate rotation velocity
    return snapPID.calculate(navX.getAxis(Axis.YAW));
  }

  public double snapRotation(double targetAngle) {
    // calculate rotation velocity
    return snapPID.calculate(navX.getAxis(Axis.YAW), targetAngle);
  }

  public boolean atSnapRotation() {
    return snapPID.atSetpoint();
  }

  public RotationMode getRotationMode() {
    return rotationMode;
  }

  public void setRotationMode(RotationMode newRotationMode) {
    // if rotation mode changed do necessary initilization
    if (rotationMode != newRotationMode) {
      if (newRotationMode == RotationMode.kZero) { // rotate to zero deg
        setSnapRotation(0.0);
      } else if (newRotationMode == RotationMode.kBar) { // rotate to bar angle
        setSnapRotation(-22.5);
      }
    }
    rotationMode = newRotationMode;
  }

  // public void setSnapRotation(double snapRotation) {
  // synchronized (lock) {
  // this.snapRotation = snapRotation;
  // }
  // }

  // public void stopSnap() {
  // synchronized (lock) {
  // this.snapRotation = Double.NaN;
  // }
  // }

  public double getAngularVelocity() {
    return navX.getRate();
  }

  public Vector2 getVelocity() {
    return velocity.getTranslationalVelocity();
  }

  public void updateKinematics() {
    HolonomicDriveSignal localSignal = new HolonomicDriveSignal(new Vector2(0, 0.05), 0, true);
  }

  public void resetDistance() {
    for (int i = 0; i < 4; i++) {
      startRotations[i] = modules[i].getCurrentDistance();
    }
  }

  public double getDistance() {
    double sum = 0;
    for (int i = 0; i < 4; i++) {
      sum += (modules[i].getCurrentDistance() - startRotations[i]);
    }
    return (sum/4.0) * (Math.PI * wheelDiameter);
  }

}
