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
  private final TalonSRX shootMotor = new TalonSRX(Constants.shootRightMotorID);
  //private final TalonSRX shootMotorRight = new TalonSRX(Constants.shootLeftMotorID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID, 
  Constants.actuatorRightPistonExtendID, Constants.actuatorRightPistonRetractID);
  private final DoubleSolenoid actuatorSolenoid2 = new DoubleSolenoid(Constants.actuatorModuleID,
  Constants.actuatorLeftPistonExtendID, Constants.actuatorLeftPistonRetractID);
  private double speed = 0;
  //private double speedRight = 0;
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {

  }
  public void shoot(double newSpeed){
    speed = newSpeed;
    //speedRight = 0.7;
  }
  // public void shootRight(){
  //   speed = 0.7;
  // }
  public void stop(){
    speed = 0;
    //speedRight = 0;
  }
  public void extendRightPistonActuator() {
    actuatorSolenoid.set(Value.kForward);
  }
  public void retractRightPistonActuator() {
    actuatorSolenoid.set(Value.kReverse);
  }
  public void extendLeftPistonActuator() {
    actuatorSolenoid2.set(Value.kForward);
  }
  public void retractLeftPistonActuator() {
    actuatorSolenoid2.set(Value.kReverse);
  }
  //public void runMotors() {
    //shootMotor.set(ControlMode.PercentOutput, speed);
    //shootMotorRight.set(ControlMode.PercentOutput, speedRight);
  //}
  @Override
  public void periodic() {
    shootMotor.set(ControlMode.PercentOutput, speed);
    //shootMotorRight.set(ControlMode.PercentOutput, speedRight);
    // This method will be called once per scheduler run
  }
  
}
