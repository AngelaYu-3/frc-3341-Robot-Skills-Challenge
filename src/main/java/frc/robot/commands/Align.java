// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Align extends CommandBase {
  /** Creates a new Align. */
  Limelight li;
  private double x;
  public Align() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    li = new Limelight();
    x= li.getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x= li.getX();
    if(x>5){
      RobotContainer.getDrive().tankDrive(-0.25, 0.25);
    }
    if(x<5){
      RobotContainer.getDrive().tankDrive(0.25, -0.25);
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
    if(x>-5&&x<5){
      RobotContainer.getDrive().tankDrive(0, 0);
      return true;
    }
    return false;
  }
}