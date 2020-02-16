/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Side;

import frc.robot.RobotContainer;

/**
 * Add your docs here.
 */
public class AutoTrajectories {
  RobotContainer mRobotContainer;
  
  public RobotContainer getRobotContainer() {
    return mRobotContainer;
  }

  // public double currentAngle = mRobotContainer.ahrs.getDisplacementX();
  // private final Trajectory autoLineToLowGoal;
  // private final Trajectory lowGoalToFrontTrench;
  // private final Trajectory frontTrenchToRearCP; //CP = Control Panel
  // private final Trajectory rearCPToFrontTrench;
  // private final Trajectory frontTrenchToLowGoal;

  // private final Trajectory lowGoaToOpponentFrontTrench;
  // private final Trajectory opponentFrontTrenchToRearTrench;
  // private final Trajectory rearTrenchToOpponentFrontTrench;
  // private final Trajectory opponentFrontTrenchToLowGoal;

  // private final Trajectory autoLineToOpponentFrontTrench;

  // private final Trajectory autoLineToSideOfSG; //SG = Shield Generator
  
  // private final Trajectory
  // private final Trajectory
  // private final Trajectory
  // private final Trajectory
  // private final Trajectory
  // private final Trajectory

  // public AutoTrajectories(TrajectoryConstraint... constraints) {
  
  //   Path autoLineToLowGoalPath = new SplinePathBuilder(Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
  //     .hermite(new Vector2(0, 90), Rotation2.ZERO)
  //     .build();

  //   // TrajectoryConstraint[] autoLineToLowGoalConstraints = {
  //   //   new MaxAccelerationConstraint(12.0), //units/s^2
  //   //   new MaxVelocityConstraint(12.0) //units/s
  //   // };
  //   autoLineToLowGoal = new Trajectory(autoLineToLowGoalPath, constraints, 1.0e-2);

  //   Path lowGoalToFrontTrenchPath = new SplinePathBuilder
  //     (new Vector2(0, 90), Rotation2.ZERO, Rotation2.ZERO)
  //     .hermite(new Vector2(66.91,-86.63), Rotation2.ZERO)
  //     .build();

  //   lowGoalToFrontTrench = new Trajectory(lowGoalToFrontTrenchPath, constraints, 1.0e-2);

  //   Path frontTrenchToRearCPPath = new SplinePathBuilder
  //     (new Vector2(0, 90), Rotation2.ZERO, Rotation2.ZERO)
  //     .hermite(new Vector2(66.91,-86.63), Rotation2.ZERO)
  //     .build();

  //   frontTrenchToRearCP = new Trajectory(autoLineToLowGoalPath, constraints, 1.0e-2);
  // }

}
