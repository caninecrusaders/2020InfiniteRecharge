/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CmdAutoDriveRotations;
import frc.robot.commands.CmdAutoDriveTime;
import frc.robot.commands.CmdAutoHighShoot;

/**
 * Add your docs here.
 */
public class AutoClassic {
  SendableChooser<Command> chooser = new SendableChooser<Command>();

  public AutoClassic() {
    SmartDashboard.putData("Auto Mode", chooser);
    chooser.setDefaultOption("Nothing", doNothing());
    chooser.addOption("OffAutoLine", offAutoLine());
    chooser.addOption("OffAutoLineHighShoot", offAutoLineHighShoot());
    chooser.addOption("OneFootForward", oneFootForward());
  }

  private Command doNothing() {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new CmdAutoDriveTime(0, 0, 0, 0)
    );
    return group;
  }

  private Command offAutoLine() {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new CmdAutoDriveTime(-0.2, 0, 0, 0.5)
    );
    return group;
  }

  private Command offAutoLineHighShoot() {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new CmdAutoDriveTime(-0.2, 0, 0, 0.5),
      new CmdAutoHighShoot(1.0, 1.0)
    );
    return group;
  }

  private Command oneFootForward() {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new CmdAutoDriveRotations(-0.2, 0, 0, 12.0)
    );
    return group;
  }

  public Command getCommand() {
    return chooser.getSelected();
  }
}
