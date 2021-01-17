// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Turn extends CommandBase {
  /** Creates a new Turn. */
  private double angle;
  private boolean left;
  
  public Turn(double angle, boolean left) {
    RobotContainer.getDrive();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveTrain.getInstance());
    this.angle = angle;
    this.left = left;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.getDrive().resetEncoders();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this can probably be optimized in some way--saw last year PID was used for power management?
    if(left){
      RobotContainer.getDrive().tankDrive(0, 0.5);
    } else {
      RobotContainer.getDrive().tankDrive(0.5,0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if(angle == RobotContainer.getDrive().getAngle()){
    return true;
  }return false;
  }
}
