package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax indexerMotor = new SparkMax(8, MotorType.kBrushless);

    private static final int CURRENT_LIMIT = 20; // amps

    private static final double FORWARD_SPEED =  0.6;
    private static final double REVERSE_SPEED = -0.4;

    public IndexerSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(CURRENT_LIMIT);

        indexerMotor.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/CurrentAmps", indexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Indexer/Output",      indexerMotor.get());
    }

    /**
     * Runs the indexer rollers forward (toward the shooter) at default speed.
     */
    public void forward() {
        indexerMotor.set(FORWARD_SPEED);
    }

    /**
     * Runs the indexer rollers in reverse (back toward intake) at default speed.
     */
    public void reverse() {
        indexerMotor.set(REVERSE_SPEED);
    }

    /**
     * Runs the indexer at an arbitrary percent output.
     * Positive = toward shooter, negative = toward intake.
     *
     * @param percent Output from -1.0 to 1.0
     */
    public void run(double percent) {
        indexerMotor.set(percent);
    }

    /**
     * Stops the indexer rollers.
     */
    public void stop() {
        indexerMotor.stopMotor();
    }
}