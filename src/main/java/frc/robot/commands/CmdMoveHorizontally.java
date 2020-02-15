/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CmdMoveHorizontally extends CommandBase {
  double distance;
  double speed;

  private RobotContainer mRobotContainer;
  private DriveTrainSubsystem mDriveTrainSubsystem;
  
  private RobotContainer getRobotContainer() {
    return mRobotContainer;
  }

  /**
   * Creates a new cmdMoveHorizontally.
   */
  public CmdMoveHorizontally(double speedIn, double distanceInFeet) {
    distance = distanceInFeet;
    speed = speedIn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveTrainSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feet = distance;

    double forward = speed;
    
    double strafe = 0;

    double rotation = 0;
    
    DriveTrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, true);
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
