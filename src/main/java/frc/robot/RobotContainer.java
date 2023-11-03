// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();
  /* Subsystem Creation */
  public static Drivetrain m_drivetrain;
  public static Gyro m_gyro;
  public static Vision m_vision;
  /* PDH in Code to allow for data logging */
  public static PowerDistribution m_pdp;
  /* Controllers */
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    m_drivetrain = new Drivetrain();
    m_gyro = new Gyro();
    m_pdp = new PowerDistribution(1, ModuleType.kRev);
    m_vision = new Vision();
    configureBindings();
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () -> m_drivetrain.drive(
                -MathUtil.applyDeadband(driverController.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
            m_drivetrain));
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  private void configureBindings() {
    driverController.a().whileTrue(new RunCommand(
        () -> m_drivetrain.setX(),
        m_drivetrain));

  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, going straight
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drivetrain::setModuleStates,
        m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, false, false));
  }


    public void disabledInit() {

    }
  

    public void enabledInit() {

    }
}
