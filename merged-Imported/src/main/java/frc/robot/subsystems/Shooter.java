package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

public class Shooter extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final SparkMax followerMotor;
    private final SparkMax feederMotor;
    private final SparkClosedLoopController shooterPID;

    private final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
    private double[] lastMapData = new double[0];

    private long spinUpStartTime = 0;
    private static final double MIN_SPINUP_TIME = 0.5;

    private boolean shooterReadyLatched = false;

    private static final double FEEDER_FIRING_VOLTAGE = 8.0;
    private static final double MIN_FIRE_VELOCITY = 1800;
    private static final double MAX_FIRE_VELOCITY = 2400;

    private final GenericEntry SPEED_MAP_ENTRY =
            Shuffleboard.getTab("Configuration")
                    .add("Shooter Speed Map", new double[]{
                            1.0, 1600.0,
                            2.1, 1600.0,
                            3.0, 1785.0,
                            4.0, 2010.0,
                            5.0, 2225.0,
                            6.0, 2438.0
                    })
                    .getEntry();

    private final GenericEntry SHOOTER_VEL =
            Shuffleboard.getTab("Configuration")
                    .add("Shooter Velocity", -1)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .getEntry();

    private final GenericEntry FIRE_BOOST_VOLTAGE =
            Shuffleboard.getTab("Configuration")
                    .add("Firing Boost Voltage", 0)
                    .withWidget(BuiltInWidgets.kNumberSlider)
                    .getEntry();

    public Shooter() {
        shooterMotor = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.closedLoop.pidf(0.0002, 0, 0, 0.0024);
        shooterConfig.smartCurrentLimit(50);
        shooterConfig.voltageCompensation(12);
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.velocityConversionFactor(1.0 / 2.0);
        shooterConfig.apply(encoderConfig);

        shooterMotor.configure(
                shooterConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );

        shooterPID = shooterMotor.getClosedLoopController();

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(shooterMotor, true);

        followerMotor.configure(
                followerConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(30);
        feederConfig.voltageCompensation(12);
        feederConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        feederConfig.inverted(true);

        feederMotor.configure(
                feederConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );
    }

    @Override
    public void periodic() {
        double[] mapData = SPEED_MAP_ENTRY.getDoubleArray(new double[0]);

        if (mapData.length >= 2 && mapData.length % 2 == 0) {
            if (!Arrays.equals(mapData, lastMapData)) {
                speedMap.clear();
                for (int i = 0; i < mapData.length; i += 2) {
                    speedMap.put(mapData[i], mapData[i + 1]);
                }
                lastMapData = mapData.clone();
            }
        }

        SmartDashboard.putNumber("Shooter/Velocity", getShooterVelocity());
        SmartDashboard.putBoolean("Shooter/ReadyLatched", shooterReadyLatched);
    }

    public void setShooterSpeed(double speed, boolean boost, boolean autoFire) {
        if (Math.abs(speed) < 1) {
            shooterPID.setSetpoint(0, SparkBase.ControlType.kVelocity);
            shooterReadyLatched = false;
            spinUpStartTime = 0;
            feederMotor.setVoltage(0);
            return;
        }

        if (spinUpStartTime == 0) {
            spinUpStartTime = System.currentTimeMillis();
        }

        double override = SHOOTER_VEL.getDouble(-1);
        if (override != -1) {
            speed = override;
        }

        double ff = boost ? FIRE_BOOST_VOLTAGE.getDouble(0) : 0.0;

        shooterPID.setSetpoint(
                speed,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ff,
                SparkClosedLoopController.ArbFFUnits.kVoltage
        );

        double currentVelocity = Math.abs(getShooterVelocity());
        double time = (System.currentTimeMillis() - spinUpStartTime) / 1000.0;

        boolean inRange = currentVelocity >= MIN_FIRE_VELOCITY && currentVelocity <= MAX_FIRE_VELOCITY;

        SmartDashboard.putNumber("Shooter/TargetSpeed", speed);
        SmartDashboard.putNumber("Shooter/CurrentVelocity", currentVelocity);
        SmartDashboard.putBoolean("Shooter/InRange", inRange);
        SmartDashboard.putNumber("Shooter/SpinupTime", time);

        if (!shooterReadyLatched && inRange && time > MIN_SPINUP_TIME) {
            shooterReadyLatched = true;
        }

        if (autoFire && shooterReadyLatched) {
            feederMotor.setVoltage(FEEDER_FIRING_VOLTAGE);
            SmartDashboard.putBoolean("Shooter/FeederRunning", true);
        } else if (autoFire) {
            feederMotor.setVoltage(0);
            SmartDashboard.putBoolean("Shooter/FeederRunning", false);
        }
    }

    public void setShooterSpeed(double speed, boolean boost) {
        setShooterSpeed(speed, boost, false);
    }

    public void setFiring(boolean manualOverride) {
        if (manualOverride || shooterReadyLatched) {
            feederMotor.setVoltage(FEEDER_FIRING_VOLTAGE);
        } else {
            feederMotor.setVoltage(0);
        }
    }

    public void testFeederDirect() {
        feederMotor.setVoltage(8.0);
    }

    public void stopFeeder() {
        feederMotor.setVoltage(0);
    }

    public void setOuttake() {
        feederMotor.setVoltage(-FEEDER_FIRING_VOLTAGE);
    }

    public void stopAll() {
        shooterPID.setSetpoint(0, SparkBase.ControlType.kVelocity);
        feederMotor.setVoltage(0);
        shooterReadyLatched = false;
        spinUpStartTime = 0;
    }

    public boolean isAtSpeed() {
        double currentVelocity = Math.abs(getShooterVelocity());
        double time = (System.currentTimeMillis() - spinUpStartTime) / 1000.0;
        return currentVelocity >= MIN_FIRE_VELOCITY && currentVelocity <= MAX_FIRE_VELOCITY && time > MIN_SPINUP_TIME;
    }

    public boolean isShooterReady() {
        return shooterReadyLatched;
    }

    public double getShooterVelocity() {
        return shooterMotor.getEncoder().getVelocity();
    }

    public double getFeederCurrent() {
        return feederMotor.getOutputCurrent();
    }

    public double getWantedVelocity(double distance) {
        return speedMap.get(distance);
    }
}