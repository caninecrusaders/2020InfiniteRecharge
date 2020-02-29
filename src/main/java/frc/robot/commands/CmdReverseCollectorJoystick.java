/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.Thrustmaster;
import frc.robot.subsystems.CollectorSubsystem;

public class CmdReverseCollectorJoystick extends CommandBase {
  private Thrustmaster mThrustmaster;
  private CollectorSubsystem mCollectorSubsystem;
  /**
   * Creates a new CmdReverseCollectorJoystick.
   */
  public CmdReverseCollectorJoystick(CollectorSubsystem collectorSubsystem, Thrustmaster thrustmaster) {
    // Use addRequirements() here to declare subsystem dependencies.
    mThrustmaster = thrustmaster;
    mCollectorSubsystem = collectorSubsystem;
    addRequirements(mCollectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -0.3;
    mCollectorSubsystem.collectFuel(speed);

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
