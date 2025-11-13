package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.BooleanSupplier;

public class SubsystemControl {
    public static Command climberControl(
            Climber climber,
            BooleanSupplier climbUp,
            BooleanSupplier climbDown,
            BooleanSupplier extendClimber
    ) {
        return Commands.run(() -> {
            if (extendClimber.getAsBoolean()) {
                climber.extendClimber();
            }

            if (climbUp.getAsBoolean()) {
                climber.startClimbingUp();
            } else if (climbDown.getAsBoolean()) {
                climber.startClimbingDown();
            } else {
                climber.stopClimbing();
            }
        }, climber);
    }

    public static Command coralControl(
            CoralScorer coralScorer,
            BooleanSupplier release,
            BooleanSupplier pointDown
    ) {
        return Commands.run(() -> {
            if (coralScorer.hasCoral()) {
                if (release.getAsBoolean()) {
                    coralScorer.setState(CoralScorer.States.DROP);
                } else {
                    if (pointDown.getAsBoolean()) {
                        coralScorer.setState(CoralScorer.States.POINT_DOWN);
                    }
                }
            } else {
                if (coralScorer.getState() != CoralScorer.States.HAS_PIECE) {
                    coralScorer.setState(CoralScorer.States.INTAKE);
                }
            }
        }, coralScorer);
    }

    public static Command elevatorControl(
            Elevator elevator,
            BooleanSupplier l2,
            BooleanSupplier l3,
            BooleanSupplier l4,
            BooleanSupplier intakePosition,
            BooleanSupplier zeroElevator
    ) {
        return Commands.run(() -> {
            if (l2.getAsBoolean()) {
                elevator.setPresetPos(1);
            } else if (l3.getAsBoolean()) {
                elevator.setPresetPos(2);
            } else if (l4.getAsBoolean()) {
                elevator.setPresetPos(3);
            } else if (intakePosition.getAsBoolean()) {
                elevator.setPresetPos(4);
            } else if (zeroElevator.getAsBoolean()) {
                elevator.zero();
            }
        }, elevator);
    }
}
