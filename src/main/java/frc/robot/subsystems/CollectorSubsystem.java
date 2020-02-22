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
import frc.robot.RobotContainer;

public class CollectorSubsystem extends SubsystemBase {
  private static CollectorSubsystem instance;
  private final TalonSRX collectorMotor = new TalonSRX(Constants.collectorMotorID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID,
  Constants.actuatorCollectorExtendID,Constants.actuatorCollectorRetractID);
  private double speed = 0;
  private boolean isExtended;
  /**
   * Creates a new CollectorSubsystem.
   */
  private CollectorSubsystem() {
    extendCollectorActuator();
  }
  public static CollectorSubsystem getInstance() {
    if(instance == null){
      instance = new CollectorSubsystem();
    }
    return instance;
  }

  public void collectFuel(double newSpeed){
    if (isExtended) {
      speed = newSpeed;
    } else {
      speed = 0;
    }
  }
  // public void releaseFuel(){
  //   speed = -0.5;
  // }
  public void stopFuel(){
    speed = 0;
  }
  public void extendCollectorActuator(){
    actuatorSolenoid.set(Value.kForward);
    isExtended = true;
  }
  public void retractCollectorActuator(){
    actuatorSolenoid.set(Value.kReverse);
    isExtended = false;
  }
  public void toggleCollector(){
    if (isExtended){
      retractCollectorActuator();
    } else {
      extendCollectorActuator();
    }
  }
  public void periodic() {
    collectorMotor.set(ControlMode.PercentOutput, 1.0);
    if(RobotContainer.isEndgame() && isExtended){
      retractCollectorActuator();
    }
    collectorMotor.set(ControlMode.PercentOutput, -speed*0.5);
    // This method will be called once per scheduler run
  }
}
