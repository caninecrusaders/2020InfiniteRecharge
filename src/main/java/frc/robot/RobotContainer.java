/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import frc.robot.input.JoystickX3D;
import frc.robot.input.Thrustmaster;
import frc.robot.input.XboxController;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.SaveZeroOffsetSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.*;

public class RobotContainer {
  public XboxController xboxRobotControl = new XboxController(2);
  public XboxController xboxDriver = new XboxController(3); // not used unless driving with xbox controller

  public JoystickX3D x3DJoystick = new JoystickX3D(0);
  public Thrustmaster thrustmasterJoystick = new Thrustmaster(1);

  public AHRS ahrs;

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final SaveZeroOffsetSubsystem saveZeroOffsetSubsystem;

  private final CmdJoystickHolonomic mCmdJoystickHolonomic;
  private final CmdTwoJoystickHolonomic mCmdTwoJoystickHolonomic;
  private final CmdXboxHolonomic mCmdXboxHolonomic;
  private final CmdRunClimbHook mCmdRunClimbHook;
  private final CmdRunCollectorXbox mCmdRunCollectorXbox;
  public final LEDStrip ledStrip;

  private final ClimberSubsystem mClimberSubsystem;
  private final CollectorSubsystem mCollectorSubsystem;
  private final ShooterSubsystem mShooterSubsystem;


  private static boolean endgame = false;

  public DriveTrainSubsystem getDriveTrainSubsystem() {
    return driveTrainSubsystem;
  }

  public SaveZeroOffsetSubsystem getSaveZeroOffsetSubsystem() {
    return saveZeroOffsetSubsystem;
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (!RobotState.isTest()) {
      saveZeroOffsetSubsystem = null;

      mCmdJoystickHolonomic = new CmdJoystickHolonomic(thrustmasterJoystick);
      mCmdTwoJoystickHolonomic = new CmdTwoJoystickHolonomic(x3DJoystick, thrustmasterJoystick);
      mCmdXboxHolonomic = new CmdXboxHolonomic(xboxDriver);
      mCmdRunClimbHook = new CmdRunClimbHook(xboxRobotControl);
      mCmdRunCollectorXbox = new CmdRunCollectorXbox(xboxRobotControl);

      ledStrip = new LEDStrip(thrustmasterJoystick);

      driveTrainSubsystem = DriveTrainSubsystem.getInstance();
      mClimberSubsystem = ClimberSubsystem.getInstance();
      mCollectorSubsystem = CollectorSubsystem.getInstance();
      mShooterSubsystem = ShooterSubsystem.getInstance();

      driveTrainSubsystem.setDefaultCommand(mCmdJoystickHolonomic);
      mClimberSubsystem.setDefaultCommand(mCmdRunClimbHook);
      // driveTrainSubsystem.setDefaultCommand(mCmdTwoJoystickHolonomic); 
      // driveTrainSubsystem.setDefaultCommand(mCmdXboxHolonomic);
      mCollectorSubsystem.setDefaultCommand(mCmdRunCollectorXbox);

      configureButtonBindings();

    } else {
      mCmdJoystickHolonomic = null;
      mCmdTwoJoystickHolonomic = null;
      mCmdXboxHolonomic = null;
      mCmdRunClimbHook = null;
      mCmdRunCollectorXbox = null;

      ledStrip = null;

      driveTrainSubsystem = null;
      mCollectorSubsystem = null;
      mClimberSubsystem = null;
      mShooterSubsystem = null;

      saveZeroOffsetSubsystem = new SaveZeroOffsetSubsystem();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxRobotControl.getAButton().whenPressed(new CmdRunLowShooter(mShooterSubsystem));
    xboxRobotControl.getYButton().whenPressed(new CmdRunHighShooter(mShooterSubsystem));

    xboxRobotControl.getBButton().whenPressed(new CmdRetractCollector(mCollectorSubsystem));
    xboxRobotControl.getXButton().whenPressed(new CmdExtendCollector(mCollectorSubsystem));

    xboxRobotControl.getStartButton().whenPressed(new CmdWinchClimb(mClimberSubsystem));
    xboxRobotControl.getLeftBumperButton().whenPressed(new CmdCancelClimb());
    xboxRobotControl.getRightBumperButton().whenPressed(new CmdClimbStartPosition());
    
    thrustmasterJoystick.getTriggerButton().whileHeld(new CmdRunCollectorJoystick(thrustmasterJoystick));
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
  
  public static void setEndgame(boolean flag){
    endgame = flag;
  }
  public static boolean isEndgame(){
    return endgame;
  }
}
