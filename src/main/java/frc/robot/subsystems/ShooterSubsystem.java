/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;
  private final TalonSRX beltMotor = new TalonSRX(Constants.beltMotorID);
  private DigitalInput detectFuel = new DigitalInput(Constants.digitalSensorIntakeID);
  private DigitalInput stopFuel = new DigitalInput(Constants.digitalSensorGateID);
  private double beltSpeed = 0;
  private double shooterSpeed = 0;
  private int fuelState = 0;
  private boolean stopCollection = false;
  private boolean shootingMode = false;

  /**
   * Creates a new ShooterSubsystem.
   */
  private ShooterSubsystem() {
    beltMotor.set(ControlMode.PercentOutput, 0);
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public void shoot(double newBeltSpeed, double newShooterSpeed) {
    beltSpeed = -newBeltSpeed;
    shooterSpeed = newShooterSpeed;
    shootingMode = true;
  }

  public void finishShooting() {
    beltMotor.set(ControlMode.PercentOutput, 0);
    beltSpeed = 0;
    fuelState = 0;
    shootingMode = false;
  }

  @Override
  public void periodic() {

    if (shootingMode) {
      beltMotor.set(ControlMode.PercentOutput, beltSpeed);
      // shootMotor.set();
    } else {
      boolean fuelIntake = !detectFuel.get(); // Sensor returns false if ball detected
      boolean stopIntake = !stopFuel.get();

      switch (fuelState) {
      case 0:
        if (fuelIntake && !stopIntake) {
          fuelState++;
        } else {
          // beltMotor.set(ControlMode.PercentOutput, 0);
        }
        break;
      case 1:
        if (!fuelIntake || stopIntake) {
          fuelState = 0;
          // beltMotor.set(ControlMode.PercentOutput, 0);
        } else {
          beltMotor.set(ControlMode.PercentOutput, -0.5);
        }
        break;
      }
      // beltMotor.set(ControlMode.PercentOutput, -0.5);
    }
  }

}
