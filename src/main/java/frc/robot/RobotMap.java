package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class RobotMap {

    //motor ports
    public static int leftDrivePort = 0;
    public static int rightDrivePort = 1;

    //joystick ports
	public static int joy = 0;
    public static int joy1 = 1;   

    //ultrasonic ports
    public static int ultrasonic1 = 0;
    public static int ultrasonic2 = 1;

    //trajectory constants--need to be found w/ robot characterization tool
    public static double kS = 1;
    public static double kV = 1;
    public static double kA = 1;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
	public static double radius = 1;;

    public static final double kTrackwidthMeters = 1;

    public static final double kMaxSpeedMetersPerSecond = 3; //should be lower than free-speed of robot
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; //apparently not that important

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    
}
