/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LowShooterSubsystem;

public class CmdRunLowShooter extends CommandBase {
  LowShooterSubsystem mShooterSubsystem;
  double endTime;

  /**
   * Creates a new cmdShooterPiston.
   */
  public CmdRunLowShooter(LowShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    mShooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooterSubsystem.retractPistonActuator();
    mShooterSubsystem.shoot(1);
    endTime = RobotController.getFPGATime();
    endTime = endTime/1000000.0 + Constants.LowShooterRunTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooterSubsystem.extendPistonActuator();
    mShooterSubsystem.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double curTime = RobotController.getFPGATime();
    curTime = curTime/1000000.0;
    if (curTime >= endTime){
      return true;
    }
    return false;
  }
}
