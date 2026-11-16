package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {

    private static final double INTAKE_VOLTAGE = 12.0;
    private static final double OUTTAKE_VOLTAGE = -12.0;

    private final SparkMax intakeSparkMax;
    private final SparkMax followerSparkMax;
    private final DoubleSolenoid intakeSwitch;
    private final Compressor compressor;

    public Intake() {
        intakeSparkMax = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);
        followerSparkMax = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);

        intakeSwitch = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 2, 3);
        compressor = new Compressor(2, PneumaticsModuleType.REVPH);
        //compressor.disable(); 

        compressor.enableAnalog(20, 30);  // Runs between 20-30 PSI

        // Start with intak3e UP
        intakeSwitch.set(DoubleSolenoid.Value.kReverse);
 
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(true);
        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        intakeConfig.smartCurrentLimit(50);
        intakeConfig.voltageCompensation(12);

        intakeSparkMax.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(intakeSparkMax, true);
        followerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        followerConfig.smartCurrentLimit(50);
        followerConfig.voltageCompensation(12);

        followerSparkMax.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        intakeSparkMax.setVoltage(0);
    }

    public Command intakeDown() {
        return Commands.runOnce(() -> intakeSwitch.set(DoubleSolenoid.Value.kForward));
    }

    public Command intakeUp() {
        return Commands.runOnce(() -> intakeSwitch.set(DoubleSolenoid.Value.kReverse));
    }

    public Command toggleIntake() {
        return Commands.runOnce(() -> {
            intakeSwitch.set(isExtended() ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        });
    }

    public boolean isExtended() {
        return intakeSwitch.get() == DoubleSolenoid.Value.kForward;
    }

    public Command intake() {
        return Commands.startEnd(
            () -> intakeSparkMax.setVoltage(INTAKE_VOLTAGE),
            () -> intakeSparkMax.setVoltage(0),
            this
        );
    }

    public Command outtake() {
        return Commands.startEnd(
            () -> intakeSparkMax.setVoltage(OUTTAKE_VOLTAGE),
            () -> intakeSparkMax.setVoltage(0),
            this
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Current", intakeSparkMax.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Output", intakeSparkMax.getAppliedOutput());
        SmartDashboard.putBoolean("Intake/Extended", isExtended());
        SmartDashboard.putNumber("Intake/Pressure", compressor.getPressure());
        SmartDashboard.putBoolean("Intake/CompressorRunning", compressor.isEnabled());
    }
}