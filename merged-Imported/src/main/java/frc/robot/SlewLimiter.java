package frc.robot;

/**
 * A simple slew-rate limiter that caps how fast a commanded value can change
 * between iterations. This is what makes a swerve robot feel planted instead of
 * jerky: when you whack the stick to full throttle, the robot ramps up to that
 * speed over several loops instead of snapping to it instantly.
 *
 * <p>WPILib 2026 removed their {@code SlewRateLimiter}, so this is a minimal
 * replacement with the same contract: call {@link #calculate(double)} every loop
 * with the value you want, and it returns the value you should actually use.
 */
public class SlewLimiter {

    private final double rateLimit; // max change per call (units/sec * dt)
    private double previousValue = 0.0;

    /**
     * @param rateLimit maximum rate of change in the commanded value's units per second
     */
    public SlewLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
    }

    /**
     * Returns the next output, clamped so it can change at most
     * {@code rateLimit * dt} from the previous output.
     *
     * @param next the desired next value
     * @return the slewed value to actually use
     */
    public double calculate(double next) {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        return calculate(next, now);
    }

    /**
     * @param next     the desired next value
     * @param curTime  current timestamp in seconds (for testing/deterministic loops)
     * @return the slewed value to actually use
     */
    public double calculate(double next, double curTime) {
        double dt = curTime - previousTime;
        if (dt < 0) {
            // Clock went backwards (e.g. simulation reset) — don't let it amplify.
            previousValue = next;
            previousTime = curTime;
            return next;
        }
        if (dt > 0.2) {
            // Huge gap (mode switch, first call after long pause) — jump straight to it
            // so we don't start from a stale value.
            previousValue = next;
            previousTime = curTime;
            return next;
        }
        double maxChange = rateLimit * dt;
        double output = edu.wpi.first.math.MathUtil.clamp(next, previousValue - maxChange,
                previousValue + maxChange);
        previousValue = output;
        previousTime = curTime;
        return output;
    }

    /** Reset the internal state so the next call starts fresh. */
    public void reset() {
        previousValue = 0.0;
        previousTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    private double previousTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
}