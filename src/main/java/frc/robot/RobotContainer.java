// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeTest;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/** Add your docs here. */
public class RobotContainer {

    // joysticks
    private static Joystick joy = new Joystick(RobotMap.joy);
    private static Joystick joy1 = new Joystick(RobotMap.joy1);
    private JoystickButton intake;

    public RobotContainer(){
        intake = new JoystickButton(joy, 1);
        intake.toggleWhenPressed(new IntakeTest());
    }

    // instantiating subsystems
    private static DriveTrain drive = new DriveTrain();


    public static Joystick getJoy() {
        return joy;
    }

    public static Joystick getJoy1() {
        return joy1;
    }

    public static DriveTrain getDrive() {
        return drive;
    }

    public SequentialCommandGroup getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        TrajectoryConfig config = new TrajectoryConfig(0.3, 0.1);
        config.setKinematics(drive.getKinematics());
        //config.setConstraint(autoVoltageConstraint);
        
        //pathweaver testing (S path)
        /*String trajectoryJSON = "paths/test.wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
         trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
         DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
       }*/
       
       Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(0.3, 0.3),
            new Translation2d(0.5, -0.3),
            new Translation2d(0.7, -0.3)
        ),
        // End 4 meters straight ahead of where we started, facing forward
        new Pose2d(0.9, 0, new Rotation2d(0)),
        // Pass config
        config
      );

        /*Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(new Pose2d(), 
              new Pose2d(1.0, 0, new Rotation2d()),
              new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
              config
         );*/
    
        RamseteCommand command = new RamseteCommand(
          trajectory,
          drive::getPose,
          new RamseteController(2, .7),
          drive.getFeedForward(),
          drive.getKinematics(),
          drive::getSpeeds,
          drive.getLeftPIDController(),
          drive.getRightPIDController(),
          drive::setOutputVolts,
          drive
        );
        //drive.resetOdometry(trajectoryEx.getInitialPose());
    
        return command.andThen(() -> drive.setOutputVolts(0, 0));
      }
}
