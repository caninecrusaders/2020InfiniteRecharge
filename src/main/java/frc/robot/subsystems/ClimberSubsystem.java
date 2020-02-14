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
  private final TalonSRX motorExtend = new TalonSRX(Constants.climbExtendMotorID);
  private final TalonSRX motorWinchOne= new TalonSRX(Constants.climbWinchMotorOneID);
  private final TalonSRX motorWinchTwo= new TalonSRX(Constants.climbWinchMotorTwoID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID,
   Constants.actuatorClimbExtendID, Constants.actuatorClimbRetractID);
  private double winchSpeed = 0;
  private double extendHookSpeed = 0;
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
    motorWinchOne.set(ControlMode.PercentOutput, winchSpeed);
    motorWinchTwo.set(ControlMode.PercentOutput, winchSpeed);
    motorExtend.set(ControlMode.PercentOutput, extendHookSpeed);
    // This method will be called once per scheduler run
  }

  public void setWinchSpeed(double speed){
    winchSpeed = speed;
  }
  public void setExtendHookSpeed(double speed){
    extendHookSpeed = speed;
  }
  public void extendClimbActuator(){
    actuatorSolenoid.set(Value.kForward);
  }
}
