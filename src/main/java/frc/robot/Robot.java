/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutoSelector;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;

  private RobotContainer mRobotContainer;
  private UsbCamera camera;

  private Command autoCommand = null;

  // private AutoTrajectories autoTrajectories = new AutoTrajectories(DriveTrainSubsystem.CONSTRAINTS); //TODO: make restraints
  // private AutoSelector autoSelector = new AutoSelector(autoTrajectories);
  // // //TODO: is this the right param?

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // camera = new UsbCamera("cam0", 0);

    camera = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
    camera.setFPS(30);
    camera.setResolution(720, 480);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    mRobotContainer = new RobotContainer();
    // CollectorSubsystem.getInstance().extendCollectorActuator();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }

    // autoCommand = autoSelector.getCommand();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.schedule();
    // }
  }

  // /**
  // * This function is called periodically during autonomous.
  // */
  // @Override
  // public void autonomousPeriodic() {
  // }

  // @Override
  // public void teleopInit() {
  // // This makes sure that the autonomous stops running when
  // // teleop starts running. If you want the autonomous to
  // // continue until interrupted by another command, remove
  // // this line or comment it out.
  // if (m_autonomousCommand != null) {
  // m_autonomousCommand.cancel();
  // }
  // }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    // Cancels all running commands at the start of test mode.
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();

    if (RobotState.isTest()) {
      if (mRobotContainer.x3DJoystick.getRawButton(2) == true
          || mRobotContainer.thrustmasterJoystick.getRawButton(13) == true) {
        mRobotContainer.getSaveZeroOffsetSubsystem().saveAllZeroOffsets();
      }
    }
  }
}
