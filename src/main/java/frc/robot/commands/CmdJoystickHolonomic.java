/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.input.Thrustmaster;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.RotationMode;

import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.drivers.NavX.Axis;

public class CmdJoystickHolonomic extends CommandBase {
  // private final DriveTrainSubsystem driveTrainSubsystem;
  private Thrustmaster joystick;
  DriveTrainSubsystem mDriveTrainSubsystem;

  private double prevPovAngle = -1.0; // not active
  /**
   * Creates a new cmdJoystickHolonomic.
   */
  public CmdJoystickHolonomic(Thrustmaster joystickIn, DriveTrainSubsystem driveTrain) {
    joystick = joystickIn;
    mDriveTrainSubsystem = driveTrain;
    addRequirements(DriveTrainSubsystem.getInstance());

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forward = -joystick.getYAxis();
    forward = Utilities.deadband(forward);
    // Square the forward stick
    forward = Math.copySign(Math.pow(forward, 2.0), forward);
    // Preferences.getInstance().putDouble("Forward", forward);

    double strafe = -joystick.getXAxis();
    strafe = Utilities.deadband(strafe);
    // Square the strafe stick
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);
    // Preferences.getInstance().putDouble("Strafe", strafe);


    double rotation;
    if (mDriveTrainSubsystem.getRotationMode() == RotationMode.kManual) {
      double newPovAngle = joystick.getPOV();
      if (newPovAngle >=0.0) { // we have hat input
        if (newPovAngle != prevPovAngle) { // if hat has changed
          if (newPovAngle > 180.0) newPovAngle -= 360.0;
          mDriveTrainSubsystem.setSnapRotation(newPovAngle);
        }
        rotation = mDriveTrainSubsystem.snapRotation();
      } else {
        rotation = -joystick.getZAxis();
        rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
        if(mDriveTrainSubsystem.isFullSpeedTurn() == false) {
          rotation = rotation/4.0;
        }
      }
      prevPovAngle = newPovAngle;
    } else {
      rotation = mDriveTrainSubsystem.snapRotation();
    }

    DriveTrainSubsystem.getInstance().drive(new Vector2(-forward, -strafe), -rotation, true);
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
