/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import com.fasterxml.jackson.databind.deser.SettableAnyProperty;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.XboxController;
import frc.robot.subsystems.LowShooterSubsystem;

public class cmdShoot extends CommandBase {
  LowShooterSubsystem mShooterSubsystem;
  XboxController xboxController;
  // double time;
  // double speed;
  /**
   * Creates a new cmdLowShoot.
   */
  public cmdShoot(XboxController controller) {
    mShooterSubsystem = LowShooterSubsystem.getInstance();
    addRequirements(mShooterSubsystem);
    xboxController = controller;

    // speed = Speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = xboxController.getY();
    //speed = Utilities.deadband 
    if (speed < 0){
      speed = 0;
    }
    mShooterSubsystem.shoot(speed);
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