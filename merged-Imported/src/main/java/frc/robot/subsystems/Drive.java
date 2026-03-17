package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

/**
 * Adapter subsystem that exposes the APIs used throughout the rest of this codebase,
 * while delegating all drivetrain control/odometry to the CTRE Phoenix 6 swerve
 * implementation (CommandSwerveDrivetrain + TunerConstants) from WRARobotics-2026.
 */
public class Drive extends SubsystemBase {

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final double maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final ApplyFieldSpeeds applySpeeds = new ApplyFieldSpeeds();
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    // Simple acceleration estimate for shooter prediction (m/s^2)
    private ChassisSpeeds lastRobotSpeeds = new ChassisSpeeds();
    private double lastSpeedSampleTime = 0.0;
    private double lastLinearAccel = 0.0;

    public Drive() {
        // PathPlanner AutoBuilder configuration
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, ff) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(8.0, 0.0, 0.0),
                        new PIDConstants(6.0, 0.0, 0.0)),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    public void zeroPose() {
        drivetrain.seedFieldCentric();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getState().Speeds;
    }

    /**
     * Field-relative chassis speeds (m/s). Useful for logging and prediction.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        var robot = getRobotRelativeSpeeds();
        return ChassisSpeeds.fromRobotRelativeSpeeds(robot, getPose().getRotation());
    }

    public Rotation2d getGyroPitch() {
        // Phoenix Pigeon 2 pitch is degrees; wrap in Rotation2d
        try {
            return Rotation2d.fromDegrees(drivetrain.getPigeon2().getPitch().getValueAsDouble());
        } catch (Exception e) {
            return Rotation2d.fromDegrees(0);
        }
    }

    /**
     * A lightweight linear acceleration estimate (m/s^2) computed from chassis speed deltas.
     */
    public double getGyroAcceleration() {
        return lastLinearAccel;
    }

    /**
     * Drive with robot-relative ChassisSpeeds.
     */
    public void drive(ChassisSpeeds speeds) {
       drivetrain.setControl(applySpeeds.withSpeeds(speeds));
    }

    /**
     * Drive with velocity commands (m/s, rad/s).
     */
    public void drive(double vxMps, double vyMps, double omegaRadPerSec, boolean fieldRelative) {
        if (fieldRelative) {
            drivetrain.setControl(fieldCentric
                    .withVelocityX(vxMps)
                    .withVelocityY(vyMps)
                    .withRotationalRate(omegaRadPerSec));
        } else {
            drivetrain.setControl(robotCentric
                    .withVelocityX(vxMps)
                    .withVelocityY(vyMps)
                    .withRotationalRate(omegaRadPerSec));
        }
    }

    /**
     * Drive with translation commands and a heading target (field-centric).
     */
    public void rotationPidDrive(double vxMps, double vyMps, double targetHeadingRad, double unused1,
            double unused2) {
        drivetrain.setControl(fieldCentricFacingAngle
                .withVelocityX(vxMps)
                .withVelocityY(vyMps)
                .withTargetDirection(Rotation2d.fromRadians(targetHeadingRad)));
    }

    /**
     * Convenience command for holding a control request.
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return drivetrain.applyRequest(request);
    }

    /**
     * The previous codebase used a final PID "snap" at the end of a path. We keep a minimal
     * placeholder here for compatibility.
     */
    public Command pidToPosition(Pose2d target) {
        return Commands.run(() -> {
            // Simple point-and-hold: drive facing target heading with zero translation
            rotationPidDrive(0.0, 0.0, target.getRotation().getRadians(), 0.0, 0.0);
        }, this).withTimeout(0.5);
    }

    /**
     * Compatibility hooks used by the old teleop drive. These are now no-ops.
     */
    public boolean shouldBumpAdjust() {
        return false;
    }

    public double closestBumpAngle() {
        return 0.0;
    }

    public void setBrakeMode(Object ignored) {
        // CTRE swerve handles neutral mode internally; keep method for compatibility.
    }

    @Override
    public void periodic() {
        // Update a simple acceleration estimate
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSpeedSampleTime;
        if (dt > 0.01) {
            var cur = getRobotRelativeSpeeds();
            double lastV = Math.hypot(lastRobotSpeeds.vxMetersPerSecond, lastRobotSpeeds.vyMetersPerSecond);
            double curV = Math.hypot(cur.vxMetersPerSecond, cur.vyMetersPerSecond);
            lastLinearAccel = Math.abs((curV - lastV) / dt);
            lastRobotSpeeds = cur;
            lastSpeedSampleTime = now;
        }
    }
}
