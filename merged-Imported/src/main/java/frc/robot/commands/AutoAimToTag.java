package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class AutoAimToTag extends Command {

    private final Drive drive;
    private final CommandXboxController controller;

    private static final double kP_omega = 5.0;
    private static final double kMaxOmega = Constants.MAX_ANGULAR_SPEED;
    private static final double DEADBAND_RAD = Math.toRadians(1.0);

    public AutoAimToTag(Drive drive, CommandXboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AutoAim/Running", true);
    }

    @Override
    public void execute() {
        var controls = RobotContainer.getControls(controller);

        // Try both empty string and the constant name
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        double tx = LimelightHelpers.getTX("limelight");

        SmartDashboard.putBoolean("AutoAim/HasTarget", hasTarget);
        SmartDashboard.putNumber("AutoAim/TX", tx);

        double omega;

        if (hasTarget) {
            double txRad = Math.toRadians(tx);

            if (Math.abs(txRad) < DEADBAND_RAD) {
                omega = 0;
                SmartDashboard.putString("AutoAim/Status", "CENTERED");
            } else {
                // Try flipping sign if it turns wrong way
                omega = MathUtil.clamp(-kP_omega * txRad, -kMaxOmega, kMaxOmega);
                SmartDashboard.putString("AutoAim/Status", "TURNING");
            }
        } else {
            omega = -controller.getRightX() * kMaxOmega;
            SmartDashboard.putString("AutoAim/Status", "NO TARGET");
        }

        SmartDashboard.putNumber("AutoAim/Omega", omega);

        //CODE TO PRINT LIMELIGHT VALUES TO DASHBOARD -----------

      
        double ty = LimelightHelpers.getTY("limelight");

        SmartDashboard.putBoolean("LL/HasTarget", hasTarget);
        SmartDashboard.putNumber("LL/TX", tx);
        SmartDashboard.putNumber("LL/TY", ty);

        //CODE TO PRINT LIMELIGHT VALUES TO DASHBOARD -----------

        drive.drive(controls.getX(), controls.getY(), omega, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AutoAim/Running", false);
        SmartDashboard.putString("AutoAim/Status", "STOPPED");
        drive.drive(0, 0, 0, true);
    }
}