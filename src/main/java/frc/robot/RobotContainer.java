// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final IndexerSubsystem m_indexer = new IndexerSubsystem();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        //auto chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous Chooser", autoChooser);
        SmartDashboard.putString("Alliance shift", DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));

        //pathplanner commands
        // run rollers until command ends
        NamedCommands.registerCommand(
            "run rollers 0.5",
            Commands.startEnd(() -> m_intake.runRollers(0.5), m_intake::stopRollers, m_intake)
        );

        // run rollers backwards until command ends
        NamedCommands.registerCommand(
            "run rollers -0.5",
            Commands.startEnd(() -> m_intake.runRollers(-0.5), m_intake::stopRollers, m_intake)
        );

        // pivot out while active
        NamedCommands.registerCommand(
            "pivot out",
            Commands.startEnd(() -> m_intake.runPivotVoltage(0.2), m_intake::stopPivot, m_intake)
        );
        //pivot in
        NamedCommands.registerCommand(
            "pivot in",
            Commands.startEnd(() -> m_intake.runPivotVoltage(-0.2), m_intake::stopPivot, m_intake)
        );

        // spin flywheel, stop all
        NamedCommands.registerCommand(
            "spin flywheel 50%",
            Commands.startEnd(() -> m_shooter.setShooterVoltage(0.5), m_shooter::stopAll, m_shooter)
        );

        // spin up, wait, feed, stop all
        NamedCommands.registerCommand(
            "shoot when ready (80 RPS)",
            Commands.sequence(
            Commands.runOnce(() -> m_shooter.setShooterRPM(80), m_shooter),
            new WaitUntilCommand(() -> m_shooter.isReady(80)),
            Commands.runOnce(() -> m_shooter.setFeederVoltage(0.6), m_shooter),
            Commands.runOnce(m_shooter::stopAll, m_shooter)
            )
        );
        // spin flywheel + sushi together, stop all
        NamedCommands.registerCommand(
            "spin flywheel + sushi",
            Commands.startEnd(() -> {
                m_shooter.setShooterRPM(80);
                m_shooter.setFeederVoltage(0.6);
            }, m_shooter::stopAll, m_shooter)
        );    
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // sysId routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Gyro reset log (start alone, not combined with SysId above)
        // Note: joystick.start() alone conflicts with the SysId combos above.
        // Removed the standalone start().onTrue to avoid binding conflict.

        // --- INTAKE ---
        // Left Trigger: run rollers inward
        joystick.leftTrigger()
            .whileTrue(new RunCommand(() -> m_intake.runRollers(0.7), m_intake))
            .onFalse(new InstantCommand(m_intake::stopRollers, m_intake));

        // Left Bumper: run rollers outward (reverse/eject)
        joystick.leftBumper()
            .whileTrue(new RunCommand(() -> m_intake.runRollers(-0.5), m_intake))
            .onFalse(new InstantCommand(m_intake::stopRollers, m_intake));

        // POV Right: pivot arm OUT/DOWN
        joystick.povRight()
            .whileTrue(new RunCommand(() -> m_intake.runPivotVoltage(0.2), m_intake))
            .onFalse(new InstantCommand(m_intake::stopPivot, m_intake));

        // POV Left: pivot arm IN/UP
        joystick.povLeft()
            .whileTrue(new RunCommand(() -> m_intake.runPivotVoltage(-0.2), m_intake))
            .onFalse(new InstantCommand(m_intake::stopPivot, m_intake));

        // --- SHOOTER ---
        // Y Button: spin flywheel 50%, then feed
        //joystick.y().whileTrue(
        //   m_shooter.run(() -> m_shooter.setShooterVoltage(0.5))
        //        .andThen(m_shooter.runEnd(() -> m_shooter.setFeederVoltage(0.6), m_shooter::stopAll))
        //);

        joystick.rightTrigger().whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> m_shooter.setShooterRPM(3000), m_shooter),
                Commands.waitUntil(() -> m_shooter.isReady(3000)),
                Commands.run(() -> {
                    m_shooter.setShooterRPM(3000);
                    m_shooter.setFeederVoltage(0.6);
                    m_indexer.run(1.0);
                }, m_shooter, m_indexer)
            )
        ).onFalse(
            Commands.runOnce(() -> {
                m_shooter.stopAll();
                m_indexer.run(0);
            }, m_shooter, m_indexer)
        );
        
        // Right Bumper: spin flywheel + sushi together immediately (no wait)
        joystick.rightBumper()
            .whileTrue(new RunCommand(() -> {
                m_shooter.setShooterRPM(4000);
                m_shooter.setFeederVoltage(0.6);
                m_indexer.run(1.0);
            }, m_shooter, m_indexer))
            .onFalse(new InstantCommand(() -> {
                m_shooter.stopAll();
                m_indexer.run(0);
            }, m_shooter, m_indexer));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}