package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAimToTag;
import frc.robot.commands.AutoHangToTag;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.*;
import static frc.robot.Utils.getSpeed2;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private final Drive drive = new Drive();
    private final Shooter shooter = new Shooter();
    private final Climb climb = new Climb(drive);
    private final Intake intake = new Intake();

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final Command shootCommand = new ShootWhileMoving(drive, shooter, m_driverController, intake);
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        NamedCommands.registerCommand("shoot command", Commands.repeatingSequence(new ShootWhileMoving(drive, shooter, m_driverController, intake)).withTimeout(6));
        NamedCommands.registerCommand("toggle intake", intake.toggleIntake());
        NamedCommands.registerCommand("intake", Commands.repeatingSequence(intake.intake()).withTimeout(3));
        configureBindings();
    }
    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(
            new RunCommand(() -> {
                var controls = getControls(m_driverController);
                var thetaInput = -m_driverController.getRightX() * MAX_ANGULAR_SPEED;
                drive.drive(controls.getX(), controls.getY(), thetaInput, true);

            }, drive));

        // m_driverController.b().whileTrue(
        // getDriveToGoal(new Pose2d(new Translation2d(14.8, 4.09),
        // Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(0))
        // );
        var restHeadingCommand = Commands.runOnce(drive::zeroPose, drive);
        restHeadingCommand.runsWhenDisabled();
        m_driverController.start().onTrue(restHeadingCommand);

        m_driverController.leftTrigger().whileTrue(intake.intake());
        m_driverController.leftBumper().whileTrue(intake.outtake());
        m_driverController.rightBumper().onTrue(intake.toggleIntake());

        m_driverController.rightTrigger().whileTrue(Commands.parallel(shootCommand));

        // --- LIMELIGHT APRILTAG ASSISTS ---
        // Hold LB for "aim only" (keeps your translation control, but rotates to face the visible tag)
        m_driverController.leftStick().whileTrue(new AutoAimToTag(drive, m_driverController));

        // Hold RB + B for "auto hang" (drives to a set distance from the visible tag, then extends climb)
        m_driverController.rightStick().whileTrue(new AutoHangToTag(drive, climb));

        m_driverController.a().whileTrue(Commands.run(
            () -> {
                boolean isAtSpeed = shooter.isAtSpeed(1700);
                shooter.setShooterSpeed(1700, false);
                shooter.setFiring(isAtSpeed);
            }, shooter));

        m_driverController.y().whileTrue(Commands.runEnd(
            () -> {
                shooter.setShooterSpeed(-1700, false);
                shooter.setOuttake();
            }, () -> {
                shooter.setShooterSpeed(0, false);
                shooter.setFiring(false);
            }, shooter));


        m_driverController.x().whileTrue(Commands.run(
            () -> {
                shooter.setShooterSpeed(1700, false);
            }, shooter));

        m_driverController.b().whileTrue(Commands.run(() -> shooter.setFiring(true)));

        // m_driverController.a().whileTrue(Commands.sequence(
        // Commands.race(
        // getDriveToGoal(new Pose2d(6, 7, Rotation2d.fromDegrees(90)),
        // Rotation2d.fromDegrees(0), normConstraints),
        // Commands.waitUntil(() ->
        // LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME) == 15)
        // ),
        // Commands.waitSeconds(0.5),
        // getDriveToGoal(new Pose2d(6, 7, Rotation2d.fromDegrees(90)),
        // Rotation2d.fromDegrees(0), normConstraints),
        // climb.extend(false)
        // ));

        // m_driverController.x().whileTrue(Commands.sequence(
        // Commands.parallel(
        // climb.setVoltageWithFeedforward(3, drive).until(() ->
        // drive.getGyroPitch().getDegrees() >= 180),
        // Commands.run(() -> drive.drive(0.2, 0, 0, true)).until(() ->
        // drive.getGyroPitch().getDegrees() >= 30)
        // ),
        // climb.fixPIDPositionReference(drive.getGyroPitch().getRadians())
        // ));

        m_driverController.povLeft().onTrue(climb.extend(false));
        m_driverController.povUp().whileTrue(climb.setVoltageWithFeedforward(12, drive, true)
            .alongWith(climb.extend(true)));
        m_driverController.povDown().onTrue(Commands.run(() -> {
            shooter.setFiring(false);
            shooter.setShooterSpeed(0, false);
        }, shooter));
        m_driverController.povDown().whileTrue(climb.setVoltageWithFeedforward(-7, drive, false));

    }

    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    }

    public static Translation2d getControls(CommandXboxController m_driverController) {

        // var yInput = Math.abs(m_driverController.getLeftY()) *
        // m_driverController.getLeftY() * MAX_LINEAR_SPEED_TELEOP;
        // var xInput = -Math.abs(m_driverController.getLeftX()) *
        // m_driverController.getLeftX() * MAX_LINEAR_SPEED_TELEOP;
        var isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        double xInput;
        double yInput;
        if (!isRed) {
            xInput = -Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY()
                * MAX_LINEAR_SPEED_TELEOP;
            yInput = -Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX()
                * MAX_LINEAR_SPEED_TELEOP;
        } else {
            xInput = Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY()
                * MAX_LINEAR_SPEED_TELEOP;
            yInput = Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX()
                * MAX_LINEAR_SPEED_TELEOP;
        }

        return new Translation2d(xInput, yInput);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        var isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        double xInput;
        double theta;
        if (isRed) {
            xInput = 0.5;
            theta = 180;
        } else {
            xInput = -0.5;
            theta = 0;
        }

        return AutoBuilder.buildAuto("New Auto");
//
//        return Commands.sequence(
//            Commands.runOnce(() -> drive.resetPose(new Pose2d(drive.getPose().getX(), drive.getPose().getY(), Rotation2d.fromDegrees(theta)))),
//            Commands.repeatingSequence(new ShootWhileMoving(drive, shooter, m_driverController, intake)).withTimeout(6)
//            //AutoBuilder.followPath(PathPlannerPath.fromPathFile("p1"))
//        );
    }

    public void disabledInit() {
        CommandScheduler.getInstance().schedule(
            Commands.runOnce(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kCoast))
                .beforeStarting(Commands.waitSeconds(3.0))
                .ignoringDisable(true));
    }

    public void enabledInit() {
        CommandScheduler.getInstance().schedule(
            Commands.runOnce(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kBrake)));
    }
}
