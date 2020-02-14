/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * Add your docs here.
 */
public class Thrustmaster extends Joystick {

  private final Button[] mButtons = { new JoystickButton(this, 1), // trigger
      new JoystickButton(this, 2), // joystick middle
      new JoystickButton(this, 3), // joystick left
      new JoystickButton(this, 4), // joystick right

      new JoystickButton(this, 5), // base bottom left **LEFT SIDE
      new JoystickButton(this, 6), // base middle left **LEFT SIDE
      new JoystickButton(this, 7), // base top left **LEFT SIDE
      new JoystickButton(this, 8), // base bottom right **LEFT SIDE
      new JoystickButton(this, 9), // base middle right **LEFT SIDE
      new JoystickButton(this, 10), // base top right **LEFT SIDE

      new JoystickButton(this, 11), // base bottom right **RIGHT SIDE
      new JoystickButton(this, 12), // base middle right **RIGHT SIDE
      new JoystickButton(this, 13), // base top right **RIGHT SIDE
      new JoystickButton(this, 14), // base top left **RIGHT SIDE
      new JoystickButton(this, 15), // base middle left **RIGHT SIDE
      new JoystickButton(this, 16) // base bottom left **RIGHT SIDE

  };

  public Thrustmaster(int port) {
    super(port);
  }

  public double getYAxis() {
    return getRawAxis(1);

  }

  public double getXAxis() {
    return getRawAxis(0);

  }

  public double getZAxis() {
    return getRawAxis(2);

  }

  public Button getTriggerButton() {
    return mButtons[1];
  }

  public Button getJoystickMiddleButton() {
    return mButtons[2];
  }

  public Button getJoystickLeftButton() {
    return mButtons[3];
  }

  public Button getJoystickRightButton() {
    return mButtons[4];
  }

  public Button getBottomLeftButtonLEFT() {
    return mButtons[5];
  }

  public Button getMiddleLeftButtonLEFT() {
    return mButtons[6];
  }

  public Button getTopLeftButtonLEFT() {
    return mButtons[7];
  }

  public Button getBottomRightButtonLEFT() {
    return mButtons[8];
  }

  public Button getMiddleRightButtonLEFT() {
    return mButtons[9];
  }

  public Button getTopRightButtonLeft() {
    return mButtons[10];
  }

  public Button getBottomRightButtonRIGHT() {
    return mButtons[11];
  }

  public Button getMiddleRightButtonRIGHT() {
    return mButtons[12];
  }
  
  public Button getTopRightButtonRIGHT() {
    return mButtons[13];
  }

  public Button getTopLeftButtonRIGHT() {
    return mButtons[14];
  }

  public Button getMiddleLeftButtonRIGHT() {
    return mButtons[15];
  }

  public Button getBottomLeftButtonRIGHT() {
    return mButtons[16];
  }
}
