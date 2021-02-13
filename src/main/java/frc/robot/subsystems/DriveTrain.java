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
  private double kGearRatio = 10;
  private double kWheelRadiusInches = 4;
  private double kTrackwidth = 28;
  private int kTiksPerRotation = 1440;
  private double kTiksToInches = 12 * Math.PI * (1/1440) * 3;
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
    left.setSelectedSensorPosition(0, 0, 10);
    right.setSelectedSensorPosition(0, 0, 10);
  }

  /*public double getUltrasonicDistance(){
    return ultrasonic.getRangeInches();
  }*/
  
  public void resetGyro(){
    gyro.zeroYaw();
  }

  public double getAngle(){
    //returns angle about Z axis from -180 to 180
    //neg left, pos right
    return gyro.getYaw();
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
      left.getSelectedSensorVelocity() * 2 * Math.PI * kWheelRadiusInches/ (kTiksPerRotation * kGearRatio),
      right.getSelectedSensorVelocity() * 2 * Math.PI * kWheelRadiusInches/ (kTiksPerRotation * kGearRatio));
  }

  public double getRightDistanceMeters(){
    return Units.inchesToMeters(right.getSelectedSensorPosition()* 12 * Math.PI * (1/1440) * 3);
   // return right.getSelectedSensorPosition(0) * 2 * Math.PI * kWheelRadiusInches/ (kTiksPerRotation * kGearRatio);
  }

  public double getLeftDistanceMeters(){
    return Units.inchesToMeters(left.getSelectedSensorPosition()* 12 * Math.PI * (1/1440) * 3);
   // return right.getSelectedSensorPosition(0) * 2 * Math.PI * kWheelRadiusInches/ (kTiksPerRotation * kGearRatio);
  }

  public double getEncoderDistance(){
    //System.out.println(Units.inchesToMeters(left.getSelectedSensorPosition(0) + right.getSelectedSensorPosition(0)) * 0.5 * kTiksToInches);
    return (left.getSelectedSensorPosition(0) + right.getSelectedSensorPosition(0)) * 0.5 * kTiksToInches;
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
    tankDrive(RobotContainer.getJoy().getY(), RobotContainer.getJoy1().getY());
    pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
  }
}
