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
//import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;
  private final TalonSRX motorWinch = new TalonSRX(Constants.climbExtendMotorID);
  private final TalonSRX motorExtend= new TalonSRX(Constants.climbWinchMotorID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID,
   Constants.actuatorClimbExtendID, Constants.actuatorClimbRetractID);
  private double speed = 0;
  private double speed2 = 0;
  /**
   * Creates a new ClimberSubsystem.
   */
  private ClimberSubsystem() {

  }
  /**
   * @return the instance
   */
  public static ClimberSubsystem getInstance() {
    if(instance == null){
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    motorWinch.set(ControlMode.PercentOutput, speed);
    motorExtend.set(ControlMode.PercentOutput, speed2);
    // This method will be called once per scheduler run
  }

  public void motorWinch(){
    speed = 0.5;
  }

  public void motorExtend(){
    speed2 = 0.5;
  }

  public void extendClimbActuator(){
    actuatorSolenoid.set(Value.kForward);
  }

  public void retractClimbActuator() {
    actuatorSolenoid.set(Value.kReverse);
  }
}
