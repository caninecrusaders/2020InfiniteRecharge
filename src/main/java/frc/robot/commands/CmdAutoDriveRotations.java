/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CmdAutoDriveRotations extends CommandBase {
  double x, y;
  double rotation;
  double endDistance;
  /**
   * Creates a new CmdAutoDriveRotations.
   */
  public CmdAutoDriveRotations(double forward, double strafe, double rotationAngle, double distance) {
    x = forward;
    y = strafe;
    rotation = rotationAngle;
    endDistance = distance;
    addRequirements(DriveTrainSubsystem.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrainSubsystem.getInstance().resetDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrainSubsystem.getInstance().drive(new Vector2(x, y), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriveTrainSubsystem.getInstance().getDistance() > endDistance) {
      DriveTrainSubsystem.getInstance().drive(Vector2.ZERO, 0, true);
      return true;
    }
    return false;
  }
}
