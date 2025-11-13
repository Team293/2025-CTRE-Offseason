// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.lib.SpikeController;
import frc.robot.commands.SubsystemControl;
import frc.robot.commands.named.PickupCoral;
import frc.robot.commands.named.ResetElevator;
import frc.robot.commands.named.SetCoralState;
import frc.robot.commands.named.SetElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.coralScorer.CoralScorer.States;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController driveController = new SpikeController(0, 0.05);
    private final CommandXboxController operatorController = new SpikeController(1, 0.05);

    // SUBSYSTEMS
    private final CommandSwerveDrivetrain drivetrain;
    private final Pneumatics pneumatics;
    private final Climber climber;
    private final CoralScorer coralScorer;
    private final Elevator elevator;
    private final Vision vision;

    public RobotContainer() {
        SignalLogger.start();

        // register subsystems
        this.drivetrain = TunerConstants.createDrivetrain();
        this.pneumatics = new Pneumatics();
        this.climber = new Climber(pneumatics);
        this.coralScorer = new CoralScorer();
        this.elevator = new Elevator();
        this.vision = new Vision(drivetrain);

        registerNamedCommands();

        this.autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Path", autoChooser);

        // warm up the path planner command
        FollowPathCommand.warmupCommand().schedule();

        configureBindings();
    }

    private void configureBindings() {
        registerSysIdBindings();
        registerDriveBindings();
        registerClimberBindings();
        registerCoralScorerBindings();
        registerElevatorBindings();
    }

    /**
     * Left DPad --> L2
     * Right DPad --> L3
     * Up DPad --> L4
     * Left or Right Bumper --> Intake
     * Down DPad --> Zero Elevator
     */
    private void registerElevatorBindings() {
        elevator.setDefaultCommand(
                SubsystemControl.elevatorControl(
                        elevator,
                        operatorController.povLeft(), // l2
                        operatorController.povRight(), // l3
                        operatorController.povUp(), // l4
                        operatorController.leftBumper().or(operatorController.rightBumper()), // intake
                        operatorController.povDown() // zero
                )
        );
    }

    /**
     * Left Trigger --> Score Coral
     * Right Trigger --> Point Down
     */
    private void registerCoralScorerBindings() {
        coralScorer.setDefaultCommand(
                SubsystemControl.coralControl(
                        coralScorer,
                        operatorController.leftTrigger(), // score
                        operatorController.rightTrigger() // point down
                )
        );
    }

    /**
     * Right Joystick --> Climber Up/Down
     * Y Button --> Extend Climber
     */
    private void registerClimberBindings() {
        climber.setDefaultCommand(
            SubsystemControl.climberControl(
                    climber,
                    () -> operatorController.getRightY() < -0.3,
                    () -> operatorController.getRightY() > 0.3,
                    operatorController.y()
            )
        );
    }

    private void registerDriveBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // brake request
        driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        driveController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void registerSysIdBindings() {
        if (Constants.ENABLE_SYSID_BINDINGS) {
            driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

            driveController.leftBumper().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            driveController.leftTrigger().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            driveController.rightBumper().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            driveController.rightTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        }
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("pickupCoral", new PickupCoral(coralScorer));
        NamedCommands.registerCommand("pickupCoral2", new PickupCoral(coralScorer));

        NamedCommands.registerCommand("pointCoralDown", new SetCoralState(coralScorer, States.POINT_DOWN));
        NamedCommands.registerCommand("pointCoralDown2", new SetCoralState(coralScorer, States.POINT_DOWN));

        NamedCommands.registerCommand("dropCoral", new SetCoralState(coralScorer, States.DROP));
        NamedCommands.registerCommand("dropCoral2", new SetCoralState(coralScorer, States.DROP));

        NamedCommands.registerCommand("resetElevator", new ResetElevator(elevator));
        NamedCommands.registerCommand("resetElevator2", new ResetElevator(elevator));

        NamedCommands.registerCommand("elevatorToL2", new SetElevator(elevator, 1));
        NamedCommands.registerCommand("elevatorToL3", new SetElevator(elevator, 2));
        NamedCommands.registerCommand("elevatorToL4", new SetElevator(elevator, 3));
//        NamedCommands.registerCommand("elevatorToL42", new SetElevator(elevator, 3)); why
        NamedCommands.registerCommand("elevatorToCS", new SetElevator(elevator, 4));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
