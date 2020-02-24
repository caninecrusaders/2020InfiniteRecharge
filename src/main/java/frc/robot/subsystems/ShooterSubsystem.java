/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  public enum ShooterMode {kLoad, kShootHi, kShootLo};

  private static ShooterSubsystem instance;
  private final TalonSRX beltMotor = new TalonSRX(Constants.beltMotorID);
  private final CANSparkMax shootMotor = new CANSparkMax(Constants.shooterMotorID, MotorType.kBrushless);
  private DigitalInput detectFuel = new DigitalInput(Constants.digitalSensorIntakeID);
  private DigitalInput stopFuel = new DigitalInput(Constants.digitalSensorGateID);

  private double beltSpeed = 0;
  private double shooterSpeed = 0;
  private int fuelLoadState = 0;
  private ShooterMode shooterMode;
  private double startTime;

  /**
   * Creates a new ShooterSubsystem.
   */
  private ShooterSubsystem() {
    finishShooting(); // initialize to load mode
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public void shoot(ShooterMode newShooterMode, double newBeltSpeed, double newShooterSpeed) {
    // Only transition to a shooting mode if not already in one (i.e. in loading mode)
    if (shooterMode == ShooterMode.kLoad) {
      startTime = RobotController.getFPGATime() / 1000000.0;
      shooterMode = newShooterMode;
      beltSpeed = -Math.abs(newBeltSpeed);
      shooterSpeed = Math.abs(newShooterSpeed);
      if (shooterMode == ShooterMode.kShootHi) shooterSpeed = -shooterSpeed; // reverse motor for hi shooter
    }
  }

  public void finishShooting() {
    // Make sure motors are off and set speeds to zero
    beltMotor.set(ControlMode.PercentOutput, 0);
    shootMotor.set(0);
    beltSpeed = 0;
    shooterSpeed = 0;

    // Init fuel load state to zero and shooter mode to loading fuel
    fuelLoadState = 0;
    shooterMode = ShooterMode.kLoad;
  }

  @Override
  public void periodic() {
    switch (shooterMode) {
      case kShootLo:
        ShootLo();
        break;
      case kShootHi:
        ShootHi();
        break;
      default:
        LoadFuel();
    }
  }

  private void LoadFuel() {
    boolean fuelIntake = !detectFuel.get(); // IR Sensor returns false if ball detected
    boolean stopIntake = !stopFuel.get();

    if (fuelLoadState == 0) { // Not loading any ball
      if (fuelIntake && !stopIntake) {
        fuelLoadState++;
      } else {
        beltMotor.set(ControlMode.PercentOutput, 0);
      }
    } else { // fuelLoadState = 1 (move ball along)
      if (!fuelIntake || stopIntake) {
        fuelLoadState = 0;
        beltMotor.set(ControlMode.PercentOutput, 0);
      } else {
        beltMotor.set(ControlMode.PercentOutput, -0.5);
      }
    }
  }

  private void ShootHi() {
    double curTime = RobotController.getFPGATime() / 1000000.0;
    double spinupTime = startTime + Constants.HiShooterSpinupTime;
    double endTime = startTime + Constants.HiShooterRunTime;
    if (curTime >=  endTime) {
      finishShooting();
    } else {
      if (curTime > spinupTime) {
        beltMotor.set(ControlMode.PercentOutput, beltSpeed);
      } else {
        beltMotor.set(ControlMode.PercentOutput, 0.0);
      }
      shootMotor.set(shooterSpeed);
    }
  }

  private void ShootLo() {
    double curTime = RobotController.getFPGATime() / 1000000.0;
    double endTime = startTime + Constants.LowShooterRunTime;
    if (curTime >=  endTime) {
      finishShooting();
    } else {
      beltMotor.set(ControlMode.PercentOutput, beltSpeed);
      shootMotor.set(shooterSpeed);
    }
  }

}      
