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
import frc.robot.RobotContainer;

public class LowShooterSubsystem extends SubsystemBase {
  private static LowShooterSubsystem instance;
  private final TalonSRX shootMotor = new TalonSRX(Constants.shootMotorID);
  //private final TalonSRX shootMotorRight = new TalonSRX(Constants.shootLeftMotorID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID, 
  Constants.actuatorPistonExtendID, Constants.actuatorPistonRetractID);
  private double speed = 0;
  private boolean isExtended;
  //private double speedRight = 0;
  /**
   * Creates a new ShooterSubsystem.
   */
  private LowShooterSubsystem() {
    shootMotor.set(ControlMode.PercentOutput, 0.5);

    extendPistonActuator();
  }
  public static LowShooterSubsystem getInstance() {
    if(instance == null){
      instance = new LowShooterSubsystem();
    }
    return instance;
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
  public void extendPistonActuator() {
    isExtended = true;
    actuatorSolenoid.set(Value.kForward);
  }
  public void retractPistonActuator() {
    if (!RobotContainer.isEndgame()){
      isExtended = false;
      actuatorSolenoid.set(Value.kReverse);
    }
  }

  public void runMotor() {
    shootMotor.set(ControlMode.PercentOutput, 0.5);

  }
  //public void runMotors() {
    //shootMotor.set(ControlMode.PercentOutput, speed);
    //shootMotorRight.set(ControlMode.PercentOutput, speedRight);
  //}
  @Override
  public void periodic() {
    shootMotor.set(ControlMode.PercentOutput, 1.0);
    if (RobotContainer.isEndgame()){
      shootMotor.set(ControlMode.PercentOutput,0);
      if(!isExtended){
        extendPistonActuator();
      }
    } else {
      shootMotor.set(ControlMode.PercentOutput, speed);
    }

    //shootMotorRight.set(ControlMode.PercentOutput, speedRight);
    // This method will be called once per scheduler run
  }
  
}
