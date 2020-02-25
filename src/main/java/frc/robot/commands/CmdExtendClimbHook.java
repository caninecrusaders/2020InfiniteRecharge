/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.input.XboxController;

public class CmdExtendClimbHook extends CommandBase {
  private ClimberSubsystem climbSubsystem;
  private XboxController xboxController;
  /**
   * Creates a new cmdExtendClimb.
   */
  public CmdExtendClimbHook(XboxController controller) {
    climbSubsystem = ClimberSubsystem.getInstance();
    addRequirements(climbSubsystem);
    xboxController = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = xboxController.getRightTriggerValue();
    climbSubsystem.setExtendHookSpeed(speed);
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
