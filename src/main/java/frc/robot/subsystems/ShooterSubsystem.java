package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterSubsystem extends SubsystemBase {
    //motors
    private final TalonFX flywheelMotor1 = new TalonFX(40);
    private final TalonFX flywheelMotor2 = new TalonFX(41);
    private final TalonFX feederMotor1 = new TalonFX(42);

    // control objects
    private final VelocityVoltage m_velocitySetter = new VelocityVoltage(0);
    private final VoltageOut m_voltageSetter = new VoltageOut(0);
    
    // shot map of distance to rpm
    private final InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem() {
        configureMotors();

        shotMap.put(1.0, 50.0);
        shotMap.put(3.0, 75.0);
        shotMap.put(5.0, 95.0);
    }

    private void configureMotors() {
        TalonFXConfiguration flyConfig = new TalonFXConfiguration();
        flyConfig.Slot0.kS = 0.637; 
        flyConfig.Slot0.kV = 0.14002;
        flyConfig.Slot0.kP = 0.11;
        flyConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flywheelMotor1.getConfigurator().apply(flyConfig);
        flywheelMotor2.getConfigurator().apply(flyConfig);

        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
        
        feederMotor1.setNeutralMode(NeutralModeValue.Brake);
    }

/**
     * Set shooter power using a percentage (-1.0 to 1.0). Internally converts to volts.
     * @param percent The percentage of full voltage (-1.0 to 1.0).
     */
    public void setShooterVoltage(double percent) {
        final double MAX_VOLTAGE = 12.0;
        flywheelMotor1.setControl(m_voltageSetter.withOutput(percent* MAX_VOLTAGE));
    }

    /**
     * Set Shooter power using rpm.
     * @param rpm The rpm to apply.
     */
    public void setShooterRPM(double rpm) {
        flywheelMotor1.setControl(m_velocitySetter.withVelocity(rpm/60.0));
    }


    /**
     * Set Feeder power using a percentage (-1.0 to 1.0). Internally converts to volts.
     * @param percent The percentage of full voltage (-1.0 to 1.0).
     */
    public void setFeederVoltage(double percent) {
        final double MAX_VOLTAGE = 12.0;
        feederMotor1.setControl(m_voltageSetter.withOutput(percent * MAX_VOLTAGE));
    }

    /**
     * Checks if the flywheel is at the target RPM (converted from RPS).
     * @param targetRPM The desired speed in Rotations Per Minute.
     */
    public boolean isReady(double targetRPM) {
        double currentRPM = flywheelMotor1.getVelocity().getValueAsDouble() * 60.0;
        return Math.abs(currentRPM - targetRPM) <= 25.0;
    }

    /**
     * Stops both the flywheel and the feeder motors immediately.
     */
    public void stopAll() {
        flywheelMotor1.stopMotor();
        feederMotor1.stopMotor();
    }

    //to be tested with limelight
    public boolean hasTarget() {
        return LimelightHelpers.getTV(""); 
    }

    public double getDistanceToTarget() {
        if (!hasTarget()) return 0.0;
        RawFiducial[] targets = LimelightHelpers.getRawFiducials("");
        return (targets.length > 0) ? targets[0].distToCamera : 0.0;
    }

    public void runFlywheelAuto() {
        double targetRPS = hasTarget() ? shotMap.get(getDistanceToTarget()) : 40.0;
        flywheelMotor1.setControl(m_velocitySetter.withVelocity(targetRPS));
    }
}