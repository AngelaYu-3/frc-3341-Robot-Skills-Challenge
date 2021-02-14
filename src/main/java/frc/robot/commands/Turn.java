// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Turn extends CommandBase {
  /** Creates a new Turn. */
  private double angle;
  
  public Turn(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getDrive());
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.getDrive().resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this can probably be optimized in some way--saw last year PID was used for power management?
    double error = Math.abs(angle - RobotContainer.getDrive().getAngle());
    double power = error * 0.01;

    if(power > 0.5) power = 0.5;
    if(angle > 0) RobotContainer.getDrive().tankDrive(power, -power);
    else RobotContainer.getDrive().tankDrive(-power, power);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //System.out.println(RobotContainer.getDrive().getAngle());
  //System.out.println(angle - RobotContainer.getDrive().getAngle());
  if(Math.abs(angle - RobotContainer.getDrive().getAngle()) <= 10){
    return true;
  }return false;
  }
}
