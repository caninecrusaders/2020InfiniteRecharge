/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonSRX shootMotorLeft = new TalonSRX(Constants.shootRightMotorID);
  private final TalonSRX shootMotorRight = new TalonSRX(Constants.shootLeftMotorID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID, 
  Constants.actuatorHatchExtendID, Constants.actuatorHatchRetractID);
  private double speedLeft = 0;
  private double speedRight = 0;
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {

  }
  public void shoot(){
    speedLeft = 0.7;
    speedRight = 0.7;
  }
  // public void shootRight(){
  //   speed = 0.7;
  // }
  public void stop(){
    speedLeft = 0;
    speedRight = 0;
  }
  public void extendHatchActuator(){
    actuatorSolenoid.set(Value.kForward);
  }
  public void retractHatchActuator(){
    actuatorSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    shootMotorLeft.set(ControlMode.PercentOutput, speedLeft);
    shootMotorRight.set(ControlMode.PercentOutput, speedRight);
    // This method will be called once per scheduler run
  }
}
