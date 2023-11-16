package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.*;


/**
 * Class regarding WPILib Trajectories, Will not be used.
 * @author Julien
 */
public class Trajectories {
    static TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);


    /**
     * Method to generate direct translation trajectories
     * @param distanceX Distance to move forwards
     * @param distanceY Distance to strafe
     * @return Returns a trajectory object containing generated trajectory
     * @author Julien
     * @since 2023-11-01
     */
    public static Trajectory translationGenerator(double distanceX, double distanceY) {
        Trajectory translate = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(distanceX/2, distanceY/2)), // Create an interior waypoint to pass through halfway through the distance.
            new Pose2d(distanceX, distanceY, new Rotation2d(0)), // End when at distance
            config);
        
        return translate;
    }

}