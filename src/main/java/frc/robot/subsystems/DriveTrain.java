// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  //Limelight li = new Limelight();
  //private Ultrasonic ultrasonic = new Ultrasonic(RobotMap.ultrasonic1, RobotMap.ultrasonic2);

  private WPI_TalonSRX left = new WPI_TalonSRX(RobotMap.leftDrivePort);
  private WPI_TalonSRX right = new WPI_TalonSRX(RobotMap.rightDrivePort);

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private static DriveTrain instance;
  
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotMap.kTrackwidth);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(RobotMap.kS, RobotMap.kV, RobotMap.kA);

  private PIDController leftPIDController = new PIDController(RobotMap.kP, 0, 0);
  private PIDController rightPIDController = new PIDController(RobotMap.kP, 0, 0);

  private double circumferenceInches = Units.metersToFeet(0.10 * Math.PI);
  private double circumference = 0.10 * Math.PI;

  Pose2d pose;

  public DriveTrain() {

    left.configFactoryDefault();
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    right.configFactoryDefault();
    right.setInverted(true);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    resetEncoders();

    gyro.reset();

    //gyro.zeroYaw();
  }

  public DriveTrain getInstance(){
    if(instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public void tankDrive(double lPower, double rPower){
    left.set(ControlMode.PercentOutput, lPower);
    right.set(ControlMode.PercentOutput, rPower);
  }

  public void arcadeDrive(double y, double x){
      //not so sensitive squaring
      if(x < 0) x = - x * x;
      else x = x * x;

      if(y < 0) y = -y * y;
      else y = y * y;

      left.set(ControlMode.PercentOutput, y + x); 
      right.set(ControlMode.PercentOutput, y - x);

  }

  public void resetEncoders(){
    left.setSelectedSensorPosition(0, 0, 10);
    right.setSelectedSensorPosition(0, 0, 10);
  }
  
  public void resetGyro(){
    gyro.reset();
  }

  public double getAngle(){
    //returns angle about Z axis from -180 to 180
    //neg left, pos right
    System.out.println("ANGLE" + gyro.getAngle());
    return gyro.getAngle();
  }

  public void resetOdometry(Pose2d initialPose) {
    resetEncoders();
    odometry.resetPosition(initialPose, gyro.getRotation2d());
 }

  /*public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }*/

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      (left.getSelectedSensorVelocity() * circumference * 10) / 4096.0,
      (right.getSelectedSensorVelocity() * circumference * 10) / 4096.0);
  }

  public double getRightDistanceMeters(){
    //System.out.println("RIGHT" + right.getSelectedSensorPosition());
    return (-right.getSelectedSensorPosition() * circumference) / 4096.0;
  }

  public double getLeftDistanceMeters(){
    //System.out.println("LEFT" + left.getSelectedSensorPosition());
    return (-left.getSelectedSensorPosition() * circumference) /4096.0;
  }

  public double getEncoderDistance(){
    return(getLeftDistanceMeters() + getRightDistanceMeters()) * 0.5;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public SimpleMotorFeedforward getFeedForward(){
    return feedForward;
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts){
    System.out.println("Left: " + -left.getSelectedSensorPosition());
    System.out.println("Right: " + -right.getSelectedSensorPosition());
    System.out.println("Pose: " + getPose());
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  @Override
  public void periodic() {
    //System.out.println(li.getDistance());
    // This method will be called once per scheduler run
    tankDrive(-RobotContainer.getJoy().getY(), -RobotContainer.getJoy1().getY());
    //arcadeDrive(RobotContainer.getJoy().getY(), RobotContainer.getJoy().getX());
    odometry.update(gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
  }
}
