package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Constants for various use around the robot. Single line comments (//) are
 * straight from the REV Code, comments that are the multi line comments (/*)
 * are written by me (Julien)
 */
public class Constants {
    /**
     * Constants that effect how the robot drives - chassis size, offsets & CAN IDs
     */
    public static final class DriveConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // Radians per Second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        /* Chassis Configuration */
        public static final double kTrackWidth = Units.inchesToMeters(25); /* Distance Between right & left wheels */
        public static final double kWheelBase = Units.inchesToMeters(25); /* Distance between front & back wheels */ // TODO:
                                                                                                                     // Measure
                                                                                                                     // out
                                                                                                                     // real
                                                                                                                     // chassis
                                                                                                                     // and
                                                                                                                     // adjust
                                                                                                                     // these
                                                                                                                     // parameters

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        /* Angular Offsets of Modules relative to Chassis in radians */
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kRearLeftChassisAngularOffset = Math.PI;
        public static final double kRearRightChassisAngularOffset = Math.PI / 2;

        /* SparkMAX CAN IDs */ // TODO: CHANGE TO REAL CAN IDs
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftDrivingCanId = 31;
        public static final int kFrontRightDrivingCanId = 21;
        public static final int kRearRightDrivingCanId = 41;

        public static final int kFrontLeftTurningCanId = 12;
        public static final int kRearLeftTurningCanId = 32;
        public static final int kFrontRightTurningCanId = 22;
        public static final int kRearRightTurningCanId = 42;

        public static final boolean kGyroReversed = false;
    }

    /**
     * Constants for the swerve module - stuff that's the same for all 4
     */
    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        /* We use the High Speed pinions, therefore there are 14T */
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        /* Math to figure out speed and position of wheel */
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1; 
        public static final double kPYController = 1;
        public static final double kPThetaController = 1; /* Controls the Rotation of the Robot during Auton */

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
    public static final class ColorConstants{
        public static final Color kBlueTarget = new Color(0.086, 0.196, 0.121);
        public static final Color kGreenTarget = new Color(0.071, 0.172, 0.051);
        public static final Color kRedTarget = new Color(0.294, 0.129, 0.023);
        public static final Color kYellowTarget = new Color(0.572, 0.6, 0.09);
    }
}
