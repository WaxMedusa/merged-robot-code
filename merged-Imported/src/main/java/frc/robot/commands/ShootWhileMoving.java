package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

public class ShootWhileMoving extends Command {

    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;
    private final CommandXboxController controller;

    private double boostTillTime = 0;

    public ShootWhileMoving(Drive drive, Shooter shooter, CommandXboxController controller, Intake intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.controller = controller;
        this.intake = intake;
        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        boostTillTime = 0;
    }

    @Override
    public void execute() {
        Translation2d robotVelocity = new Translation2d(
                drive.getFieldRelativeSpeeds().vxMetersPerSecond,
                drive.getFieldRelativeSpeeds().vyMetersPerSecond);

        Translation2d robotPositionT0 = drive.getPose().getTranslation();

        var linearVelocity = robotVelocity.getNorm();
        var linearAccel = drive.getGyroAcceleration();

        double timeTillStop = (linearAccel < 0.05) ? 0.3 : Math.min(0.3, linearVelocity / linearAccel);

        Translation2d targetPos = RobotContainer.isRed() ? Constants.HUB_POS_RED : Constants.HUB_POS_BLUE;

        Translation2d robotShootPosition = robotPositionT0.plus(robotVelocity.times(timeTillStop));
        Rotation2d targetRotation = targetPos.minus(robotShootPosition).getAngle();

        // Use odometry distance for stability during initial testing
        var robotPose = drive.getPose();
        double distance = robotPose.getTranslation().getDistance(targetPos);

        // To test Limelight distance, uncomment these lines:
        // double limelightDist = RobotContainer.getDistanceToTarget();
        // if (limelightDist > 0) distance = limelightDist;

        double wantedShooterVelocity = shooter.getWantedVelocity(distance);

        var controls = RobotContainer.getControls(controller);
        drive.rotationPidDrive(controls.getX(), controls.getY(), targetRotation.getRadians(), 0, 0);

        double angleError = MathUtil.angleModulus(
                drive.getPose().getRotation().minus(targetRotation).getRadians());

        boolean isAligned = Math.abs(angleError) < Constants.SHOOT_ANGLE_RANGE_RAD;
        boolean isAtSpeed = shooter.isAtSpeed();

        if (shooter.getFeederCurrent() > Constants.FEEDER_HAS_BALL_CURRENT_THRESHOLD) {
            boostTillTime = Timer.getFPGATimestamp() + Constants.SHOOT_BOOST_TIME_S;
        }

        boolean useBoost = boostTillTime > Timer.getFPGATimestamp();
        shooter.setShooterSpeed(wantedShooterVelocity, useBoost);

        boolean shouldFire = isAligned && isAtSpeed && distance < 2.5 && linearVelocity < 0.1;
        shooter.setFiring(shouldFire);

        Logger.recordOutput("Shooter/Wanted Velocity", wantedShooterVelocity);
        Logger.recordOutput("Shooter/Angle Error", angleError);
        Logger.recordOutput("Shooter/Is Aligned", isAligned);
        Logger.recordOutput("Shooter/Is At Speed", isAtSpeed);
        Logger.recordOutput("Shooter/Ready To Fire", shouldFire);
            Logger.recordOutput("Shooter/Distance", distance);
            Logger.recordOutput("Shooter/Linear Velocity", linearVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFiring(false);
    }
}