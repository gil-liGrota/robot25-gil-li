package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.transfer.transfer;

public class transferCommands {

    public static Command setVoltage(transfer transfer, double voltage) {
        return Commands.startEnd(() -> transfer.getIO().setVoltage(voltage),
                () -> transfer.getIO().stopMotor(), transfer);
    }

    public static Command autoIntake(transfer transfer) {
        return Commands.startEnd(() -> transfer.getIO().setVoltage(3),
                () -> transfer.getIO().stopMotor(), transfer).until(transfer.getIO()::isCoralIn);
    }
}
