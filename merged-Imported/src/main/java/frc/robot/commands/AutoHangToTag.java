package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;

/**
 * Drives the robot to a consistent "hang" approach pose relative to a visible AprilTag.
 * 
 * How it works:
 * - Uses Limelight target pose in ROBOT SPACE (meters) when an AprilTag is visible.
 * - Drives forward/back to a desired standoff distance (x).
 * - Strafes to center on the tag (y -> 0).
 * - Rotates to minimize tx.
 * - When within tolerance for a short hold time, it extends the climber.
 */
public class AutoHangToTag extends Command {
    private final Drive drive;
    private final Climb climb;

    // Tunables (start here, then tune on the practice field)
    private static final double kP_x = 1.8; // (m/s) per meter error
    private static final double kP_y = 2.2;
    private static final double kP_omega = 5.0; // rad/s per rad error

    private static final double kDesiredX = 0.75; // meters in front of tag
    private static final double kTolX = 0.08;
    private static final double kTolY = 0.08;
    private static final double kTolTxDeg = 2.0;

    private static final double kMaxV = Constants.MAX_LINEAR_SPEED_TELEOP;
    private static final double kMaxOmega = Constants.MAX_ANGULAR_SPEED;

    private static final double kHoldTime = 0.25;
    private double stableSince = -1;
    private boolean extended = false;

    public AutoHangToTag(Drive drive, Climb climb) {
        this.drive = drive;
        this.climb = climb;
        addRequirements(drive, climb);
    }

    @Override
    public void initialize() {
        stableSince = -1;
        extended = false;
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(Constants.LIMELIGHT_NAME);
        if (!hasTarget) {
            // No target: stop for safety
            drive.drive(0, 0, 0, true);
            stableSince = -1;
            return;
        }

        Pose3d targetRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_NAME);
        double x = targetRobotSpace.getX(); // forward (m)
        double y = targetRobotSpace.getY(); // left (m)

        double xErr = x - kDesiredX;
        double yErr = y - 0.0;

        double vx = MathUtil.clamp(kP_x * xErr, -kMaxV, kMaxV);
        double vy = MathUtil.clamp(kP_y * yErr, -kMaxV, kMaxV);

        double txDeg = LimelightHelpers.getTX(Constants.LIMELIGHT_NAME);
        double omega = MathUtil.clamp(-kP_omega * Math.toRadians(txDeg), -kMaxOmega, kMaxOmega);

        drive.drive(vx, vy, omega, true);

        boolean stable = Math.abs(xErr) < kTolX && Math.abs(yErr) < kTolY && Math.abs(txDeg) < kTolTxDeg;
        double now = Timer.getFPGATimestamp();
        if (stable) {
            if (stableSince < 0) stableSince = now;
        } else {
            stableSince = -1;
        }

        if (!extended && stableSince > 0 && (now - stableSince) > kHoldTime) {
            // Extend once we are aligned and at distance
            climb.extend(false).schedule();
            extended = true;
        }
    }

    @Override
    public boolean isFinished() {
        return extended;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);
    }
}
