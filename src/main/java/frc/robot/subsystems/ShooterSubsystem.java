/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonSRX shootMotor = new TalonSRX(Constants.shooterMotorID);
  private final TalonSRX shootMotor2 = new TalonSRX(Constants.shootMotorID);
  private double speed = 0;
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {

  }
  public void shoot(){
    speed = 0.7;
  }
  public void shooter(){
    speed = 0.7;
  }
  public void stop(){
    speed = 0;
  }
  @Override
  public void periodic() {
    shootMotor.set(ControlMode.PercentOutput, speed);
    shootMotor2.set(ControlMode.PercentOutput, speed);
    // This method will be called once per scheduler run
  }
}
