/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.NavX;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CmdZeroRobot extends CommandBase {
  private DriveTrainSubsystem mDriveTrainSubsystem;
  private NavX navX;
  /**
   * Creates a new CmdZeroRobot.
   */
  public CmdZeroRobot(NavX gyro) {
    navX = gyro;
    mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
    addRequirements(mDriveTrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // mDriveTrainSubsystem.snapRotationController.calculate(navX.getAngle(), dt);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDriveTrainSubsystem.snapRotationController.setSetpoint(0);
    // mDriveTrainSubsystem.drive(Vector2.ZERO, rotationalVelocity, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
