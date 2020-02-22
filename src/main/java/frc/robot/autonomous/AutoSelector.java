/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.util.Side;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CmdFollowTrajectory;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * Add your docs here.
 */
public class AutoSelector {
  private final AutoTrajectories trajectories;

  private static SendableChooser<Side> sideChooser;
  private static SendableChooser<Rotation2> orientationChooser;
  // private static SendableChooser<AutonomousMode> autonomousModeChooser;

  // static {
  // ShuffleboardTab autoTab = Shuffleboard.getTab("Auto")
  // }
  public AutoSelector(AutoTrajectories trajectories) {
    this.trajectories = trajectories;
  }

  private Command makeASquare() {
    // AutonomousMode mode = autonomousModeChooser.getSelected();
    // Rotation2 startingOrientation = orientationChooser.getSelected();

    SequentialCommandGroup group = new SequentialCommandGroup(
        new CmdFollowTrajectory(trajectories.testThreeFeetForward),
        new CmdFollowTrajectory(trajectories.testThreeFeetForwardAndThreeFeetRight),
        new CmdFollowTrajectory(trajectories.testThreeFeetLeft), 
        new CmdFollowTrajectory(trajectories.testToZero));
    return group;
  }

  public Command getCommand() {
    Rotation2 startingOrientation = Rotation2.ZERO;


    SequentialCommandGroup group = new SequentialCommandGroup(
      new InstantCommand(() -> {
        DriveTrainSubsystem.getInstance().gyroscope.setAdjustmentAngle(
                DriveTrainSubsystem.getInstance().gyroscope.getUnadjustedAngle().rotateBy(startingOrientation)
        );
    }));
    group.runsWhenDisabled();

    // Set the gyro angle to the correct starting angle
    // group.addSequential(new InstantCommand(() -> {
    //     DriveTrainSubsystem.getInstance().getGyroscope().setAdjustmentAngle(
    //             DriveTrainSubsystem.getInstance().getGyroscope().getUnadjustedAngle().rotateBy(startingOrientation)
    //     );
    // }));

    // If we want to manually drive the robot, return now.


    return group;
  }
}
