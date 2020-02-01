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

public class CollectorSubsystem extends SubsystemBase {
  private final TalonSRX collectorMotor = new TalonSRX(Constants.collectorMotorID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID,
  Constants.actuatorCollectorExtendID,Constants.actuatorCollectorRetractID);
  private double speed = 0;
  /**
   * Creates a new CollectorSubsystem.
   */
  public CollectorSubsystem() {

  }
  public void collectFuel(){
    speed = 1;
  }
  public void releaseFuel(){
    speed = -0.5;
  }
  public void stopFuel(){
    speed = 0;
  }
  public void extendCollectorActuator(){
    actuatorSolenoid.set(Value.kForward);
  }
  public void retractCollectorActuator(){
    actuatorSolenoid.set(Value.kReverse);
  }
  public void periodic() {
    collectorMotor.set(ControlMode.PercentOutput, speed);
    // This method will be called once per scheduler run
  }
}
