// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;

//autos class
public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.driveArcade(() -> 0, () -> .1).withTimeout(.1),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
        ballSubsystem.spinUpCommand().withTimeout(1),
        ballSubsystem.launchCommand().withTimeout(9),
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()));
  }

    // Example autonomous command which drives forward for 1 second.
    public static final Command centerToBackBalls(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
      return new SequentialCommandGroup(
          // Drive backwards for .25 seconds. The driveArcadeAuto command factory
          // creates a command which does not end which allows us to control
          // the timing using the withTimeout decorator
          driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2),
          // Stop driving. This line uses the regular driveArcade command factory so it
          // ends immediately after commanding the motors to stop
          driveSubsystem.driveArcade(() -> 0, () -> .1).withTimeout(.1),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
          // total of 10 seconds
         ballSubsystem.spinUpCommand().withTimeout(1),
          ballSubsystem.launchCommand().withTimeout(5),
          // Stop running the launcher
          ballSubsystem.runOnce(() -> ballSubsystem.stopCommand().withTimeout(.1)),
          driveSubsystem.turn90Left(), 

          new ParallelDeadlineGroup(
              // The "Deadline" (The intake stops when this drive sequence is done)
              new SequentialCommandGroup(
                  driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2.0),
                  driveSubsystem.turn90Left().withTimeout(1.0),  driveSubsystem.turn90Right(),  
                  driveSubsystem.driveArcade(() -> .5, () -> 0). withTimeout(1.5)
              ),
                // The "Background" command
                ballSubsystem.intakeCommand()
              ) //end of ParallelDeadLineGroup
      );//end of SequentialGroup

    } //end of centerToBackBalls
  
}
