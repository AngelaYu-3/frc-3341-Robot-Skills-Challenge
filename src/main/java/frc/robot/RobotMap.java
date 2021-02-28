package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class RobotMap {

    //motor ports
    public static int leftDrivePort = 2;
    public static int rightDrivePort = 3;

    //joystick ports
	public static int joy = 0;
    public static int joy1 = 1;   

    //ultrasonic ports
    public static int ultrasonic1 = 0;
    public static int ultrasonic2 = 1;

    //mp constants
    public static double kP = 1.71;
    public static double kS = 0.709;
    public static double kA = 0.598;
    public static double kV = 4.7; 
    public static double trackwidth = 0.63;
    
}
