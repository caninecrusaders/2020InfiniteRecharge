/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.util.Side;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

/**
 * Add your docs here.
 */
public class AutoSelector {
  private final AutoTrajectories trajectories;

  private static SendableChooser<Side> sideChooser;
  private static SendableChooser<Rotation2> orientationChooser;
  // private static SendableChooser<AutonomousMode> autonomousModeChooser;

  // static {
  //   ShuffleboardTab autoTab = Shuffleboard.getTab("Auto")
  // }
  public AutoSelector(AutoTrajectories trajectories) {
    this.trajectories = trajectories;
  }

  private Command makeASquare() {
    // AutonomousMode mode = autonomousModeChooser.getSelected();
    // Rotation2 startingOrientation = orientationChooser.getSelected();

    CommandGroupBase group = new CommandGroupBase(){
    
      @Override
      public void addCommands(Command... commands) {
        group.
      }
    };

    group.addSequential(new FollowTrajectoryCommand(trajectories.testThreeFeetForward));


  }
}
