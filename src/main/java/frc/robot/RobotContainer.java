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
import frc.robot.input.JoystickX3D;
import frc.robot.input.Thrustmaster;
import frc.robot.input.XboxController;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.SaveZeroOffsetSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.LowShooterSubsystem;
import frc.robot.commands.*;

public class RobotContainer {

  public XboxController xboxDriverOne = new XboxController(2);
  public XboxController xboxDriverTwo = new XboxController(3);

  public JoystickX3D joystickDriverOne = new JoystickX3D(0);
  public Thrustmaster joystickDriverTwo = new Thrustmaster(1);

  public AHRS ahrs;

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final SaveZeroOffsetSubsystem saveZeroOffsetSubsystem;

  private final CmdJoystickHolonomic mCmdJoystickHolonomic;
  private final CmdTwoJoystickHolonomic mCmdTwoJoystickHolonomic;
  private final CmdXboxHolonomic mCmdXboxHolonomic;
  public final LEDStrip ledStrip;

  private final ClimberSubsystem mClimberSubsystem;
  private final CollectorSubsystem mCollectorSubsystem;
  private final LowShooterSubsystem mLowShooterSubsystem;

  private final cgClimb mCgClimb;

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

      mCmdJoystickHolonomic = new CmdJoystickHolonomic(joystickDriverOne);
      mCmdTwoJoystickHolonomic = new CmdTwoJoystickHolonomic(joystickDriverOne, joystickDriverTwo);
      mCmdXboxHolonomic = new CmdXboxHolonomic(xboxDriverOne);
      ledStrip = new LEDStrip();


      driveTrainSubsystem = DriveTrainSubsystem.getInstance();
      mClimberSubsystem = ClimberSubsystem.getInstance();
      mCollectorSubsystem = CollectorSubsystem.getInstance();
      mLowShooterSubsystem = LowShooterSubsystem.getInstance();
      driveTrainSubsystem.setDefaultCommand(mCmdJoystickHolonomic);
      // driveTrainSubsystem.setDefaultCommand(mCmdTwoJoystickHolonomic); 
      // driveTrainSubsystem.setDefaultCommand(mCmdXboxHolonomic);

      mCgClimb = new cgClimb(mClimberSubsystem);
      CmdDefaultCollector collectFuel = new CmdDefaultCollector( xboxDriverTwo);
      mCollectorSubsystem.setDefaultCommand(collectFuel);
      CmdDefaultLowShooter shootFuel = new CmdDefaultLowShooter( xboxDriverTwo);
      mLowShooterSubsystem.setDefaultCommand(shootFuel);
      configureButtonBindings();

    } else {
      driveTrainSubsystem = null;
      mCmdJoystickHolonomic = null;
      mCmdTwoJoystickHolonomic = null;
      mCmdXboxHolonomic = null;
      mCgClimb = null;
      ledStrip = null;
      mCollectorSubsystem = null;
      mClimberSubsystem = null;
      mLowShooterSubsystem = null;
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
    xboxDriverTwo.getAButton().whenPressed(new CmdRunLowShooter(mLowShooterSubsystem));
    xboxDriverTwo.getYButton().whenPressed(new CmdStartEndgame());
    xboxDriverTwo.getStartButton().whenPressed(new CmdExtendCollector(mCollectorSubsystem));
    xboxDriverTwo.getBackButton().whenPressed(new CmdRetractCollector(mCollectorSubsystem));
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
