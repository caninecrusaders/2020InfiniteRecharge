/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDStrip extends SubsystemBase {
  public Spark blinkin = new Spark(Constants.blinkinController);
  //12V LED STRIP
  // http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip() {
    // rainbowPartyPalette();
    strobeRed();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // rainbowPartyPalette();
    strobeRed();
  }

  public void rainbowPartyPalette() {
    blinkin.set(-0.97);
  }

  public void rainbowRainbowPalette() {
    blinkin.set(-0.99);
  }

  public void rainbowOceanPalette() {
    blinkin.set(-0.95);
  }

  public void strobeRed() {
    blinkin.set(-0.17);
  }

}

