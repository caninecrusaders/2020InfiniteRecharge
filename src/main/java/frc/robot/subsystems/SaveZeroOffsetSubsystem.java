/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SaveZeroOffsetSubsystem extends SubsystemBase {
  AnalogInput offsetFL = new AnalogInput(Constants.encoderFL);
  AnalogInput offsetFR = new AnalogInput(Constants.encoderFR);
  AnalogInput offsetBL = new AnalogInput(Constants.encoderBL);
  AnalogInput offsetBR = new AnalogInput(Constants.encoderBR);
  /**
   * Creates a new SaveZeroOffsetSubsystem.
   */
  public SaveZeroOffsetSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void saveAllZeroOffsets() {
    Preferences.getInstance().putDouble("Front Left Offset", volt2rad(offsetFL.getAverageVoltage()));
    Preferences.getInstance().putDouble("Front Right Offset", volt2rad(offsetFR.getAverageVoltage()));
    Preferences.getInstance().putDouble("Back Left Offset", volt2rad(offsetBL.getAverageVoltage()));
    Preferences.getInstance().putDouble("Back Right Offset", volt2rad(offsetBR.getAverageVoltage()));
  }

  private double volt2rad(double volt) {
    // return (1.0 - volt / RobotController.getVoltage5V()) * 2.0 * Math.PI;
    return (volt / RobotController.getVoltage5V()) * 2.0 * Math.PI;
  }
}
