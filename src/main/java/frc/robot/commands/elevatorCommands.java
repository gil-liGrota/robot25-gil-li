package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.elevator.elevator;

public class elevatorCommands {
    public static Command goToPosition(elevator elevator, double position) {
        return new FunctionalCommand(() -> {
            System.out.println("going to position: " + position);
            elevator.getIO().stopMotor();
            elevator.getIO().resetPID(position);
        },
                () -> elevator.getIO().setGoal(position),
                interrupted -> {
                    elevator.getIO().stopMotor();
                    if (interrupted) {
                        System.out.println("interupted go to: " + position);
                    }
                },
                elevator.getIO().atGoal(), elevator);
    }

    public static Command stopElevator(elevator elevator) {
        return Commands.runOnce(() -> elevator.getIO().stopMotor(), elevator);
    }

    public static Command closeUntilSwitch(elevator elevator) {
        return Commands.run(() -> elevator.getIO().setVoltage(-0.5), elevator)
                .until(elevator.getIO()::isPressed);
    }

    public static Command closeElevator(elevator elevator) {
        return Commands.sequence(
                elevatorCommands.goToPosition(elevator, 0),
                elevatorCommands.closeUntilSwitch(elevator));
    }

    public static Command closeElevatorManual(elevator elevator) {
        return Commands.run(() -> elevator.getIO().setVoltage(4), elevator);
    }

    public static Command openElevatorManual(elevator elevator) {
        return Commands.run(() -> elevator.getIO().setVoltage(-1), elevator);
    }
}
