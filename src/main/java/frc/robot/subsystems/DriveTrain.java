// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private TalonSRX left = new TalonSRX(RobotMap.leftDrivePort);
  private TalonSRX right = new TalonSRX(RobotMap.rightDrivePort);
  private AHRS gyro = new AHRS();
  private Ultrasonic ultrasonic = new Ultrasonic(RobotMap.ultrasonic1, RobotMap.ultrasonic2);
  private static DriveTrain instance;
  private double TIKS_TO_INCHES;
  
  public DriveTrain() {
    left.setInverted(true);

    left.configFactoryDefault();
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    right.configFactoryDefault();
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    ultrasonic.setAutomaticMode(true);

    gyro.calibrate();
  }

  public static DriveTrain getInstance(){
    if(instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public void tankDrive(double lPower, double rPower){
    left.set(ControlMode.PercentOutput, lPower);
    right.set(ControlMode.PercentOutput, rPower);
  }

  public void resetEncoders(){
    left.setSelectedSensorPosition(0,0,10); 
    right.setSelectedSensorPosition(0,0,10); 
  }

  public double getDistance(){
    return ((left.getSelectedSensorPosition(0) + right.getSelectedSensorPosition(0))/2) * TIKS_TO_INCHES;
  }

  public double getUltrasonicDistance(){
    return ultrasonic.getRangeInches();
  }
  
  public void resetGyro(){
    gyro.zeroYaw();
  }

  public double getAngle(){
    //returns angle about Z axis from -180 to 180
    //neg left, pos right
    return gyro.getYaw();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tankDrive(RobotContainer.getJoy().getY(), RobotContainer.getJoy().getY());
  }
}
