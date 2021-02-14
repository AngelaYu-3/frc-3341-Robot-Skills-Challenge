// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeTest extends CommandBase {
  /** Creates a new IntakeTest. */
  private TalonSRX storer = new TalonSRX(10);
  
  public IntakeTest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //when ball is intook, goes from 0 to about 5 amps
    storer.set(ControlMode.PercentOutput, 0.5);
    System.out.println(storer.getSupplyCurrent()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storer.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
