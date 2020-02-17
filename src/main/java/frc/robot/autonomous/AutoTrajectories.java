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

  public double currentAngle = mRobotContainer.ahrs.getDisplacementX();
  private final Trajectory testThreeFeetForward;
  private final Trajectory testThreeFeetForwardAndThreeFeetRight;
  private final Trajectory testThreeFeetLeft;
  private final Trajectory testToZero;

  private final Trajectory autoLineToLowGoal;
  private final Trajectory lowGoalToFrontTrench;
  private final Trajectory frontTrenchToRearCP; // CP = Control Panel
  private final Trajectory rearCPToFrontTrench;
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

  public AutoTrajectories(TrajectoryConstraint... constraints) {
    Path testThreeFeetForwardPath = new SplinePathBuilder
    (Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
    .hermite(new Vector2(0,36), Rotation2.ZERO)
    .build();

    testThreeFeetForward = new Trajectory(testThreeFeetForwardPath, constraints, 1.0e-2);

    Path testThreeFeetForwardAndThreeFeetRightPath = new SplinePathBuilder
    (new Vector2(0, 36), Rotation2.ZERO, Rotation2.ZERO)
    .hermite(new Vector2(36,36), Rotation2.ZERO)
    .build();

    testThreeFeetForwardAndThreeFeetRight = new Trajectory(testThreeFeetForwardAndThreeFeetRightPath, constraints, 1.0e-2);

    Path testThreeFeetLeftPath = new SplinePathBuilder
    (new Vector2(36, 36), Rotation2.ZERO, Rotation2.ZERO)
    .hermite(new Vector2(36,0), Rotation2.ZERO)
    .build();

    testThreeFeetLeft = new Trajectory(testThreeFeetLeftPath, constraints, 1.0e-2);

    Path testToZeroPath = new SplinePathBuilder
    (new Vector2(36, 0), Rotation2.ZERO, Rotation2.ZERO)
    .hermite(new Vector2(0,0), Rotation2.fromDegrees(90.0))
    .build();

    testToZero = new Trajectory(testToZeroPath, constraints, 1.0e-2);




    Path autoLineToLowGoalPath = new SplinePathBuilder
    (Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
      .hermite(new Vector2(0, 90), Rotation2.ZERO)
      .build();

    // TrajectoryConstraint[] autoLineToLowGoalConstraints = {
    //   new MaxAccelerationConstraint(12.0), //units/s^2
    //   new MaxVelocityConstraint(12.0) //units/s
    // };
    autoLineToLowGoal = new Trajectory(autoLineToLowGoalPath, constraints, 1.0e-2);

    Path lowGoalToFrontTrenchPath = new SplinePathBuilder
      (new Vector2(0, 90), Rotation2.ZERO, Rotation2.ZERO)
      .hermite(new Vector2(66.91,-86.63), Rotation2.ZERO)
      .build();

    lowGoalToFrontTrench = new Trajectory(lowGoalToFrontTrenchPath, constraints, 1.0e-2);

    Path frontTrenchToRearCPPath = new SplinePathBuilder
      (new Vector2(66.91, -86.63), Rotation2.ZERO, Rotation2.ZERO)
      .hermite(new Vector2(0,-172.27), Rotation2.ZERO)
      .build();

    frontTrenchToRearCP = new Trajectory(frontTrenchToRearCPPath, constraints, 1.0e-2);

    Path rearCPToFrontTrenchPath = new SplinePathBuilder
    (new Vector2(0, -172.27), Rotation2.ZERO, Rotation2.ZERO)
    .hermite(new Vector2(0, 172.27), Rotation2.ZERO)
    .build();

    rearCPToFrontTrench = new Trajectory(rearCPToFrontTrenchPath, constraints, 1.0e-2);
    // Path frontTrenchToLowGoalPath = new SplinePathBuilder
    // (new Vector2(0, y), initialHeading, initialRotation)
  }

}
