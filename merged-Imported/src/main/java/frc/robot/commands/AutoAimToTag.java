package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

/**
 * Hold-to-run command that rotates the robot to center the currently visible AprilTag.
 * Translation remains under driver control.
 */
public class AutoAimToTag extends Command {

    private final Drive drive;
    private final CommandXboxController controller;

    // Tunables
    private static final double kP_omega = 5.0; // rad/s per rad of tx error
    private static final double kMaxOmega = Constants.MAX_ANGULAR_SPEED;

    public AutoAimToTag(Drive drive, CommandXboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var controls = RobotContainer.getControls(controller);

        boolean hasTarget = LimelightHelpers.getTV(Constants.LIMELIGHT_NAME);
        double omega;
        if (hasTarget) {
            // tx is degrees; convert to radians for controller
            double txRad = Math.toRadians(LimelightHelpers.getTX(Constants.LIMELIGHT_NAME));
            omega = MathUtil.clamp(-kP_omega * txRad, -kMaxOmega, kMaxOmega);
        } else {
            omega = -controller.getRightX() * kMaxOmega;
        }

        drive.drive(controls.getX(), controls.getY(), omega, true);
    }
}
