package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Logging;
import frc.robot.subsystems.SwerveSubsystem;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

public class SwervePathCommand extends CommandBase {

    private final SwerveSubsystem subsystem;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final RamseteController ramsete;
    private final Trajectory trajectory;
    private Instant start;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwervePathCommand(SwerveSubsystem subsystem)
    {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.SWERVE_LENGTH/2.0, Constants.SWERVE_WIDTH/2.0),
                new Translation2d(Constants.SWERVE_LENGTH/2.0, -Constants.SWERVE_WIDTH/2.0),
                new Translation2d(-Constants.SWERVE_LENGTH/2.0, Constants.SWERVE_WIDTH/2.0),
                new Translation2d(-Constants.SWERVE_LENGTH/2.0, -Constants.SWERVE_WIDTH/2.0)
        );
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(subsystem.getYaw()), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        ramsete = new RamseteController();
        trajectory = createPath();
        Logging.log.info("Trajectory\n");
        Logging.log.info(trajectory + "");
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.subsystem.initialize();
        this.subsystem.resetGyro();
        start = Instant.now();
    }

    public ChassisSpeeds calculate(Pose2d pose, Trajectory.State state){
        Transform2d trans = new Transform2d(pose, state.poseMeters);
        double mag = Math.sqrt(Math.pow(trans.getX(), 2) + Math.pow(trans.getY(), 2)) * state.velocityMetersPerSecond;
        return new ChassisSpeeds(mag * trans.getX(), mag * trans.getY(), 0);
    }


    int count = 0;
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d pose = odometry.update(Rotation2d.fromDegrees(subsystem.getYaw()), subsystem.getStates());
        double elapsed = Duration.between(start, Instant.now()).toMillis() / 1000.0;
        Trajectory.State state = trajectory.sample(elapsed);
        ChassisSpeeds speeds = calculate(pose, state);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        subsystem.setStates(states);

        if(count++ % 10 == 0) {
            Logging.log.info("Elapsed Time");
            Logging.log.info(elapsed + "");
            Logging.log.info("Yaw");
            Logging.log.info(subsystem.getYaw() + "");
            Logging.log.info("Pose");
            Logging.log.info(pose + "");
            Logging.log.info("State");
            Logging.log.info(state + "");
            Logging.log.info("Speeds");
            Logging.log.info(speeds + "");
            Logging.log.info("States");
            Logging.log.info(states[0] + "");
            Logging.log.info(states[1] + "");
            Logging.log.info(states[2] + "");
            Logging.log.info(states[3] + "");
        }

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.setHeadings(0);
        subsystem.setVelocities(0);
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    public static Trajectory createPath(){
        double maxVelocityMetersPerSecond = 1;
        double maxAccelerationMetersPerSecondSq = .5;

        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(1), Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(2), Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(3), Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(4), Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(5), Rotation2d.fromDegrees(0)));

        TrajectoryConfig config = new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

        return TrajectoryGenerator.generateTrajectory(waypoints, config);

//        Pose2d start = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0));
//        Pose2d end = new Pose2d(Units.inchesToMeters(32) - Units.feetToMeters(8), Units.feetToMeters(15) - Units.feetToMeters(1), Rotation2d.fromDegrees(-90));
//
//        List<Translation2d> waypoints = new ArrayList<>();
//        waypoints.add(new Translation2d(Units.inchesToMeters(0), Units.feetToMeters(2)));
//        waypoints.add(new Translation2d(Units.inchesToMeters(32), Units.feetToMeters(7)));
//        waypoints.add(new Translation2d(Units.inchesToMeters(32), Units.feetToMeters(15)));
//        waypoints.add(new Translation2d(Units.inchesToMeters(0), Units.feetToMeters(15)));
//
//        TrajectoryConfig config = new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
//
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
//
//        return trajectory;
    }
}
