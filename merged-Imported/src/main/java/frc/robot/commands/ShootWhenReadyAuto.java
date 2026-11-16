package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootWhenReadyAuto extends Command {

    private final Shooter shooter;
    private final double targetRPM;
    private final double durationSeconds;

    private long startTime;

    public ShootWhenReadyAuto(Shooter shooter, double rpm, double seconds) {
        this.shooter = shooter;
        this.targetRPM = rpm;
        this.durationSeconds = seconds;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        shooter.setShooterSpeed(targetRPM, true); 
    }

    @Override
    public void execute() {
        shooter.setShooterSpeed(targetRPM, true, true);
        shooter.setFiring(shooter.isAtSpeed()); 
    }

    @Override
    public boolean isFinished() {
        double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        return elapsed > durationSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeed(0, false);
        shooter.setFiring(false);
    }
}