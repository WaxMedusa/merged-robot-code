package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // PathPlanner and drive tolerances
    public static final double PATH_FINISH_CLOSE_DISTANCE_M = 0.2;
    public static final double PATH_FINISH_CLOSE_DISTANCE_M_PID = 0.03;
    public static final double ANGLE_CLOSE_RAD = Math.PI / 90;

    // Vision
    public static final String LIMELIGHT_NAME = "limelight";

    // Drive speeds
    public static final double MAX_LINEAR_SPEED_AUTO = 2.0;
    public static final double MAX_LINEAR_SPEED_TELEOP = 2.5;
    public static final double MAX_ANGULAR_SPEED = Math.PI * 0.75;

    // Teleop input shaping — tune these from the Shuffleboard "Drive" tab.
    // Deadband: sticks below this value are treated as zero (kills stick creep).
    public static final double DRIVE_DEADBAND = 0.08;
    // Response curve exponent. 1.0 = linear, <1.0 = more sensitive at low speed,
    // >1.0 = less sensitive at low speed (cubic = 3.0). 2.0 = quadratic (current behavior).
    public static final double DRIVE_INPUT_EXPONENT = 2.0;
    // Max rate of change of the commanded velocity (m/s per 20ms loop, rad/s per 20ms loop).
    // This is what stops the robot from jerking when you whack the stick.
    public static final double DRIVE_SLEW_RATE_LINEAR = 3.0;   // m/s per iteration
    public static final double DRIVE_SLEW_RATE_ANGULAR = 4.0; // rad/s per iteration

    // Shooter
    public static final double SHOOT_ANGLE_RANGE_RAD = Units.degreesToRadians(2.0);

    // Limelight distance calculation
    public static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;
    public static final double LIMELIGHT_HEIGHT_M = 0.1;
    public static final double HUB_TAG_HEIGHT_M = 1.124;

    // Fixed close-range shot
    public static final double CLOSE_RANGE_RPM = 3300.0;
    // Shooting / feeder
    public static final double FEEDER_HAS_BALL_CURRENT_THRESHOLD = 30.0;
    public static final double SHOOT_BOOST_TIME_S = 0.5;

    // HUB positions
    public static final Translation2d HUB_POS_RED = new Translation2d(
            Units.inchesToMeters(469.11),
            Units.inchesToMeters(158.84)
    );

    public static final Translation2d HUB_POS_BLUE = new Translation2d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(158.84)
    );

    // Path constraints
    public static final PathConstraints normConstraints =
            new PathConstraints(MAX_LINEAR_SPEED_AUTO, 5.0, MAX_ANGULAR_SPEED, 6 * Math.PI);
}