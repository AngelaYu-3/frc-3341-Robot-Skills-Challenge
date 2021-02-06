// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends CommandBase {
  /** Creates a new DriveForward. */
  private double distance;
  private final Timer m_timer = new Timer();
  private double time;
  private double initLeft, initRight;

  public DriveForward(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    RobotContainer.getDrive().resetEncoders();
   // initLeft = RobotContainer.getDrive().getLeftDistanceMeters();
    //initRight = RobotContainer.getDrive().getRightDistanceMeters();
    m_timer.reset();
    this.time = time;
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    RobotContainer.getDrive().resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*System.out.println("Left:" + RobotContainer.getDrive().getLeftDistanceMeters() + "Right: " + RobotContainer.getDrive().getRightDistanceMeters());
    if(m_timer.get() < time){
      //System.out.println("forwards");
      RobotContainer.getDrive().tankDrive(-0.3, -0.3);
    }*/
      if(distance > 0){
        RobotContainer.getDrive().tankDrive(-0.3, -0.3);
      }else{
        RobotContainer.getDrive().tankDrive(0.3, 0.3);
      } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //91.5 inches
    if(distance == RobotContainer.getDrive().getDistance()){
      /*double LeftD = RobotContainer.getDrive().getLeftDistanceMeters() - initLeft;
      double RightD = RobotContainer.getDrive().getRightDistanceMeters() - initRight;
      System.out.println("Left: " + LeftD + "Right: " + RightD);*/
      RobotContainer.getDrive().tankDrive(0, 0);
      return true;
    }return false;
  }
}
