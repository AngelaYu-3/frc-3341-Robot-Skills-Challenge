// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveForward extends CommandBase {

  double targetDistance;

  /** Creates a new DriveForward. */
  public DriveForward(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getDrive());
    targetDistance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("drive forward");
    RobotContainer.getDrive().resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetDistance < 0) {
      RobotContainer.getDrive().tankDrive(-0.5, -0.5);
    } else if (targetDistance > 0) {
      RobotContainer.getDrive().tankDrive(0.5, 0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(targetDistance - RobotContainer.getDrive().getEncoderDistance());
    if (Math.abs(Math.abs(targetDistance) - Math.abs(RobotContainer.getDrive().getEncoderDistance())) <= 0.1) {
      return true;
    }
    return false;
  }
}