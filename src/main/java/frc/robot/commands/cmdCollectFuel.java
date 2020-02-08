/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.swing.text.Utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.XboxController;
import frc.robot.subsystems.CollectorSubsystem;

public class cmdCollectFuel extends CommandBase {
  private CollectorSubsystem collectorSubsystem;
  private XboxController xboxController;
  /**
   * Creates a new cmdCollectFuel.
   */
  public cmdCollectFuel(CollectorSubsystem collectSubsystem, XboxController controller) {
    addRequirements(collectSubsystem);
    collectorSubsystem = collectSubsystem;
    xboxController = controller;

   
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
    collectorSubsystem.collectFuel(speed);
    //collectorSubsystem.releaseFuel();
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
