/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.stream.Collector;

import com.kauailabs.navx.frc.AHRS;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.Thrustmaster;
import frc.robot.subsystems.CollectorSubsystem;

public class CmdRunCollectorJoystick extends CommandBase {
  private Thrustmaster mThrustmaster;
  private CollectorSubsystem collectorSubsystem;


  /**
   * Creates a new CmdRunCollectorJoystick.
   */
  public CmdRunCollectorJoystick() {
    collectorSubsystem = CollectorSubsystem.getInstance();
    addRequirements(collectorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.5;
    double forward = mThrustmaster.getYAxis();
    double strafe = mThrustmaster.getXAxis();
    // double averageValue = (forward + strafe) / 2;
    Vector2 vector = new Vector2(forward, strafe);
    double vectorSpeed = vector.length;
    if(Math.abs(vectorSpeed) > 0 ) {
      speed = (vectorSpeed/2);
      collectorSubsystem.collectFuel(speed);
    } else {
      collectorSubsystem.collectFuel(speed);
    }
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
