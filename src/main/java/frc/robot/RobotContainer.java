// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();
  /* Subsystem Creation */
  public static Drivetrain m_drivetrain;
  public static Gyro m_gyro;
  public static Vision m_vision;
  /* PDH in Code to allow for data logging*/
  public static PowerDistribution m_pdp;
  /* Controllers */
  private final CommandXboxController driverController = new CommandXboxController(0);
  public RobotContainer() {
    m_drivetrain = new Drivetrain();
    m_gyro = new Gyro();
    m_pdp = new PowerDistribution(1, ModuleType.kRev);
    m_vision = new Vision();
    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
