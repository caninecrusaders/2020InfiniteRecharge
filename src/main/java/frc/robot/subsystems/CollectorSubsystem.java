/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.frcteam2910.common.robot.drivers.NavX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotController;
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
  private boolean motorStalled = false;
  private double stallStartTime = 0;
  
  /**
   * Creates a new CollectorSubsystem.
   */
  private CollectorSubsystem() {
    extendCollectorActuator();
    motorStalled = false;
    stallStartTime = 0;
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
    // This method will be called once per scheduler run
    if(RobotContainer.isEndgame() && isExtended){
      retractCollectorActuator();
    }

    // collectorMotor.set(ControlMode.PercentOutput, -speed*0.5);
    setMotorSpeed();
  }

  /*--------------------------------------------------------------*
   * This checks the motor current to see if it is stalled. If the
   * current exceeds a set limit for more than 0.5 sec it is 
   * considered stalled. If the current drops back below the limit
   * the time is reset. This assumes the motorStalled flag is not
   * true and will just return motorStalled flag if it is true.
   *--------------------------------------------------------------*/
  private boolean isMotorStalled() {
    if (!motorStalled) {
      if (collectorMotor.getStatorCurrent() > 10.0) {
        double curTime = RobotController.getFPGATime();
        if (stallStartTime == 0.0) {
          stallStartTime = curTime;
        } else if (curTime > stallStartTime + 0.5) {
          motorStalled = true;
        }
      } else {
        motorStalled = false;
        stallStartTime = 0.0;
      }
    }
    return motorStalled;
  }

  /*----------------------------------------------------------------*
   * This sets the motor speed as follows
   * 1. If the endgame has started stop the motor
   * 2. Else if the motor is stalled reverse the motor for 1 sec
   * 3. Otherwise, check if the motor is stalled. If stalled turn off
   *    the motor (it will reverse the next time thru). If not 
   *    stalled, the speed based on the speed input.
   *----------------------------------------------------------------*/
  private void setMotorSpeed() {
    if (RobotContainer.isEndgame()) {
      collectorMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      if (motorStalled) { // reverse motor for 1 sec
        double curTime = RobotController.getFPGATime();
        if (curTime < stallStartTime + 1.5) { // motorStalled not set for 0.5 sec
          collectorMotor.set(ControlMode.PercentOutput, 1.0);
        } else { // reset motor stalled flag and time and start collecting again
          motorStalled = false;
          stallStartTime = 0.0;
        }
      } else { // normal operation
        if (isMotorStalled()) {
          collectorMotor.set(ControlMode.PercentOutput, 0.0);
        } else {
          collectorMotor.set(ControlMode.PercentOutput, -speed*0.5);
        }
      }
    }
  }

}
