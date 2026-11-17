package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;

public class Utils {

    /**
     * Applies a deadband and a response curve to a joystick axis.
     *
     * <p>Sticks are rarely perfectly centered, so a small deadband prevents the robot
     * from "creeping" when you think you're stopped. The response curve then shapes how
     * sensitive the robot is at low vs high stick input.
     *
     * @param raw      raw stick value, -1..1
     * @param deadband sticks below this (in magnitude) are treated as zero
     * @param exponent 1.0 = linear, 2.0 = quadratic, 3.0 = cubic. Values &lt; 1.0
     *                make the low end MORE sensitive; values &gt; 1.0 make it LESS.
     * @return shaped value in -1..1, preserving sign
     */
    public static double shapeJoystick(double raw, double deadband, double exponent) {
        if (Math.abs(raw) < deadband) {
            return 0.0;
        }
        // Rescale so the curve starts at the deadband edge instead of jumping from 0.
        double sign = Math.copySign(1.0, raw);
        double mag = Math.abs(raw);
        double scaled = (mag - deadband) / (1.0 - deadband);
        return sign * Math.pow(scaled, exponent);
    }

    public static double getSpeed2(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond + chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond;
    }

    /**
     * Checks if a point is within a rectangular boundary.
     *
     * @param point       The Translation2d position to check.
     * @param minBoundary The bottom-left corner of the bounding box.
     * @param maxBoundary The top-right corner of the bounding box.
     * @return True if the point is inside or on the edge of the box.
     */
    public static boolean isPointInBox(Translation2d point, Translation2d minBoundary, Translation2d maxBoundary) {
        return point.getX() >= minBoundary.getX() &&
                point.getX() <= maxBoundary.getX() &&
                point.getY() >= minBoundary.getY() &&
                point.getY() <= maxBoundary.getY();
    }
}
