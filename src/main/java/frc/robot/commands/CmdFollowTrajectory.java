/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import org.frcteam2910.common.control.Trajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class CmdFollowTrajectory extends CommandBase {
  private final Supplier<Trajectory> trajectorSupplier;

  private Trajectory trajectory;

  
  /**
   * Creates a new CmdFollowTrajectory.
   */
  public CmdFollowTrajectory(Trajectory trajectory) {
    this(() -> trajectory);
    // Use addRequirements() here to declare subsystem dependencies.

  }
  public CmdFollowTrajectory(Supplier<Trajectory> trajectorySupplier) {
    this.trajectorSupplier = trajectorySupplier;
    addRequirements(DriveTrainSubsystem.getInstance());
    this.runsWhenDisabled();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = trajectorSupplier.get();
    DriveTrainSubsystem.getInstance().updateKinematics();
    // DriveTrainSubsystem.getInstance().getFollower().follow(trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //DriveTrainSubsystem.getInstance().setSnapRotation(trajectory.calculateSegment(trajectory.getDuration()).rotation.toRadians());
    //TODO: need to add this

  }

  // @Override
  // public void interruped() {
  //   end(true);
  //   getDriveTrainSubsystem().getFollower().cancel();
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; /*DriveTrainSubsystem.getInstance().getFollower().getCurrentTrajectory().isEmpty();*/
  }
}
