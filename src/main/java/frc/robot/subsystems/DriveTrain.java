// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
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
  private double kTrackwidth = 28;
  private double kS = 1;
  private double kV = 1;
  private double kA = 1;
  private double kP = 1;
  private double kI = 0;
  private double kD = 0;

  private TalonSRX left = new TalonSRX(RobotMap.leftDrivePort);
  private TalonSRX right = new TalonSRX(RobotMap.rightDrivePort);

  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  //private Ultrasonic ultrasonic = new Ultrasonic(RobotMap.ultrasonic1, RobotMap.ultrasonic2);

  private static DriveTrain instance;
  
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(kTrackwidth));
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

  private PIDController leftPIDController = new PIDController(kP, kI, kD);
  private PIDController rightPIDController = new PIDController(kP, kI, kD);

  Pose2d pose = new Pose2d();

  public DriveTrain() {
    //left.setInverted(true);

    left.configFactoryDefault();
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    right.configFactoryDefault();
    right.setInverted(true);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    resetEncoders();

   //ultrasonic.setAutomaticMode(true);

    gyro.calibrate();
  }

  public DriveTrain getInstance(){
    if(instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public void tankDrive(double lPower, double rPower){
   System.out.println("Left: " + lPower);
   System.out.println("Right: " + rPower);

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

  /*public double getUltrasonicDistance(){
    return ultrasonic.getRangeInches();
  }*/
  
  public void resetGyro(){
    gyro.reset();
  }

  public double getAngle(){
    //returns angle about Z axis from -180 to 180
    //neg left, pos right
    System.out.println("ANGLE" + gyro.getAngle());
    return gyro.getAngle();
  }

  public void setOutput(double leftVolts, double rightVolts){
    left.set(ControlMode.PercentOutput, leftVolts/12);
    right.set(ControlMode.PercentOutput, rightVolts/12);
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  //convert from tiks/s to m/s
  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      left.getSelectedSensorVelocity() * 0.15 * Math.PI * (1/4096.0),
      right.getSelectedSensorVelocity() * 0.15 * Math.PI * (1/4096.0));
  }

  public double getRightDistanceMeters(){
    //System.out.println("RIGHT" + right.getSelectedSensorPosition());
    return right.getSelectedSensorPosition() * 0.40 * Math.PI * (1/4096.0);
    //return right.getSelectedSensorPosition() * kTicksToMeters;
  }

  public double getLeftDistanceMeters(){
    //System.out.println("LEFT" + left.getSelectedSensorPosition());
    return left.getSelectedSensorPosition() * 0.40 * Math.PI * (1/4096.0);
    //return left.getSelectedSensorPosition() * kTicksToMeters;
  }

  public double getEncoderDistance(){
    return(getLeftDistanceMeters() + getRightDistanceMeters()) * 0.5;
    //return (Math.abs(getLeftDistanceMeters()) + Math.abs(getRightDistanceMeters()) * 0.5);
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getPose(){
    return pose;
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
    left.set(ControlMode.PercentOutput, leftVolts/12);
    right.set(ControlMode.PercentOutput, rightVolts/12);
  }

  public void reset(){
    odometry.resetPosition(new Pose2d(), getHeading());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //tankDrive(-RobotContainer.getJoy().getY(), -RobotContainer.getJoy1().getY());
    //arcadeDrive(RobotContainer.getJoy().getY(), RobotContainer.getJoy().getX());
    pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
  }
}
