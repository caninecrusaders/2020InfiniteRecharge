/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.input.Thrustmaster;

public class LEDStrip extends SubsystemBase {
  public Spark blinkin = new Spark(Constants.blinkinController);
  double beatsPerMinuteRainbowPalette = -0.69;
  double beatsPerMinutePartyPalette = -0.67;
  double beatsPerMinuteOceanPalette = -0.65;
  double beatsPerMinuteLavaPalette = -0.63;
  double beatsPerMinuteForestPalette = -0.61;

  double rainbowPartyPalette = -0.97;
  double rainbowRainbowPalette = -0.99;
  double rainbowOceanPalette = -0.95;
  double rainbowLavePalette = -0.93;
  double rainbowForestPalette = -0.91;
  double rainbowGlitter = -0.89;
  double strobeRed = -0.17;
  double larsonScannerRed = -0.35;
  double heartbeatSlow = 0.03;
  double heartbeatMedium = 0.05;
  double heartbeakFast = 0.07;
  double colorWavesColor1and2 = 0.53;
  double blue = 0.87;
  double blueViolet = 0.89;
  double violet = 0.91;
  double confetti = -0.87;
  double shotRed = -0.85;
  double shotBlue = -0.83;
  double shotWhite = -0.81;
  double sinelonRainbowPalette = -0.79;
  double sinelonPartyPalette = -0.77;
  double sinelonOceanPalette = -0.75;
  double sinelonLavaPalette = -0.73;
  double sinelonForestPalette = -0.71;
  double red = 0.61;

  //12V LED STRIP
  // http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  /**
   * Creates a new LEDStrip.
   */
  // public RobotContainer mRobotContainer;

  // public RobotContainer getRobotContainer() {
  //   return mRobotContainer;
  // }

  private Thrustmaster mThrustmaster;

  public LEDStrip(Thrustmaster thrustmaster) {
    mThrustmaster = thrustmaster;
    // rainbowPartyPalette();
    // setBlinkinPattern(beatsPerMinuteOceanPalette);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Slider Value", this.mThrustmaster.getSliderAxis());
    // This method will be called once per scheduler run
    setDefaultColor();
    // setBlinkinPattern(shotWhite);
    // everyOption();
  }

  public void setBlinkinPattern(double value) {
    blinkin.set(value);
  }

  public void setDefaultColor() {
    if (this.mThrustmaster.getSliderAxis() > -0.1) {
      setBlinkinPattern(blue);
    } else {
      setBlinkinPattern(red);
    }
  }

  public void everyOption() {
    blinkin.set(this.mThrustmaster.getSliderAxis());
  }

  // public void rainbowPartyPalette() {
  //   blinkin.set(-0.97);
  // }

  // public void rainbowRainbowPalette() {
  //   blinkin.set(-0.99);
  // }

  // public void rainbowOceanPalette() {
  //   blinkin.set(-0.95);
  // }
  
  // public void rainbowLavePalette() {
  //   blinkin.set(-0.93);
  // }

  // public void rainbowForestPalette() {
  //   blinkin.set(-0.91);
  // }

  // public void rainbowGlitter() {
  //   blinkin.set(-0.89);
  // }

  // public void strobeRed() {
  //   blinkin.set(-0.17);
  // }

  // public void larsonScannerRed() {
  //   blinkin.set(-0.35);
  // }

  // public void heartbeatSlow() {
  //   blinkin.set(0.03);
  // }

  // public void heartbeatMedium() {
  //   blinkin.set(0.05);
  // }

  // public void heartbeakFast() {
  //   blinkin.set(0.07);
  // }

  // public void colorWavesColor1and2() {
  //   blinkin.set(0.53);
  // }

  // public void blue() {
  //   blinkin.set(0.87);
  // }

  // public void blueViolet() {
  //   blinkin.set(0.89);
  // }

  // public void violet() {
  //   blinkin.set(0.91);
  // }

  // public void confetti() {
  //   blinkin.set(-0.87);
  // }

  // public void shotRed() {
  //   blinkin.set(-0.85);
  // }
  
  // public void shotBlue() {
  //   blinkin.set(-0.83);
  // }

  // public void shotWhite() {
  //   blinkin.set(-0.81);
  // }

  // public void sinelonRainbowPalette() {
  //   blinkin.set(-0.79);
  // }

  // public void sinelonPartyPalette() {
  //   blinkin.set(-0.77);
  // }

  // public void sinelonOceanPalette() {
  //   blinkin.set(-0.75);
  // }

  // public void sinelonLavaPalette() {
  //   blinkin.set(-0.73);
  // }

  // public void sinelonForestPalette() {
  //   blinkin.set(-0.71);
  // }


}

