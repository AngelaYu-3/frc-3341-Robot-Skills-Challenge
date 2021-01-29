// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.Barrel;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class RobotContainer {
    
    //joysticks
    private static Joystick joy = new Joystick(RobotMap.joy);
    private static Joystick joy1 = new Joystick(RobotMap.joy1);

    //instantiating subsystems
    private static DriveTrain drive = new DriveTrain();
    private static Barrel barrel = new Barrel();

    public static Joystick getJoy(){
        return joy;
    }

    public static Joystick getJoy1(){
        return joy1;
    }

    public static DriveTrain getDrive(){
        return drive;
    }

	/*public Command getAutonomousCommand() {

        //takes in max velocity and acceleration
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));  
        config.setKinematics(drive.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            //list of waypoints
            Arrays.asList(new Pose2d(), new Pose2d(1, 0, new Rotation2d())),
            config    
        );

        RamseteCommand command = new RamseteCommand(
            trajectory, drive::getPose, new RamseteController(2.0, 0.7), 
            drive::getFeedForward, drive.getKinematics(), drive.getSpeeds(), 
           drive.getLeftPIDController(), drive.getRightPIDController(), drive::setOutput, drive);

        return command;

    }*/
}
