/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.cmdJoystickHolonomic;
import frc.robot.commands.cmdTwoJoystickHolonomic;
import frc.robot.commands.cmdXboxHolonomic;
import frc.robot.input.JoystickX3D;
import frc.robot.input.XboxController;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.SaveZeroOffsetSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public XboxController xboxDriverOne = new XboxController(2);
  public XboxController xboxDriverTwo = new XboxController(3);

  public JoystickX3D joystickDriverOne = new JoystickX3D(0);
  public JoystickX3D joystickDriverTwo = new JoystickX3D(1);

  public AHRS ahrs;

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final SaveZeroOffsetSubsystem saveZeroOffsetSubsystem;
  private final cmdJoystickHolonomic mCmdJoystickHolonomic;
  private final cmdTwoJoystickHolonomic mCmdTwoJoystickHolonomic;
  private final cmdXboxHolonomic mCmdXboxHolonomic;

  public DriveTrainSubsystem getDriveTrainSubsystem() {
    return driveTrainSubsystem;
  }

  public SaveZeroOffsetSubsystem getSaveZeroOffsetSubsystem() {
    return saveZeroOffsetSubsystem;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(boolean isTestMode) {
    if (!isTestMode) {
      mCmdJoystickHolonomic = new cmdJoystickHolonomic(joystickDriverOne);
      mCmdTwoJoystickHolonomic = new cmdTwoJoystickHolonomic(joystickDriverOne, joystickDriverTwo);
      mCmdXboxHolonomic = new cmdXboxHolonomic(xboxDriverOne);
      saveZeroOffsetSubsystem = null;

      driveTrainSubsystem = DriveTrainSubsystem.getInstance();
      driveTrainSubsystem.setDefaultCommand(mCmdJoystickHolonomic);
      // driveTrainSubsystem.setDefaultCommand(mCmdTwoJoystickHolonomic);
      // driveTrainSubsystem.setDefaultCommand(mCmdXboxHolonomic);

    } else {
      driveTrainSubsystem = null;
      mCmdJoystickHolonomic = null;
      mCmdTwoJoystickHolonomic = null;
      mCmdXboxHolonomic = null;
      saveZeroOffsetSubsystem = new SaveZeroOffsetSubsystem();
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An ExampleCommand will run in autonomous
  // return m_autoCommand;
  // }
}
