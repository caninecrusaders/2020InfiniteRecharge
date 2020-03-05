/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CmdAutoDrive extends CommandBase {
  private double x , y;
  private double rotation;
  private double startTime;
  private double time;
  
  /**
   * Creates a new CmdAutoDrive.
   */
  public CmdAutoDrive(double x, double y, double rotationAngle, double seconds) {
    this.x = x;
    this.y = y;
    rotation = rotationAngle;
    time = seconds;
    addRequirements(DriveTrainSubsystem.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = RobotController.getFPGATime()/1000000.0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrainSubsystem.getInstance().drive(new Vector2(x, y), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrainSubsystem.getInstance().drive(Vector2.ZERO, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double curTime = RobotController.getFPGATime()/1000000.0;
    if (curTime > startTime + time) {
      return true;
    }
    return false;
  }
}
