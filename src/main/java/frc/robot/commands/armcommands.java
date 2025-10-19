package frc.robot.commands;
import static frc.robot.subsystems.arm.armConstance.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.arm.arm;
import frc.robot.subsystems.arm.armIO;

public class armcommands {
    public static Command goToPosition(arm arm, double goal) {
        armIO io = arm.getIO();
        return new FunctionalCommand(() -> io.resetPID(goal), () -> io.setGoal(goal),
                (interrupted) -> io.resistGravity(), io.atGoal(), arm)
                .withName("Move arm to " + goal);
    }

    public static Command closeArm(arm coralArm) {
        return Commands.sequence(
                goToPosition(coralArm, CLOSE_ARM_POSITION),
                setVoltage(coralArm, -0.5).until(coralArm.getIO()::getLowSwitch)).withName("close arm");
    }

    public static Command setVoltage(arm coralArm, double voltage) {
        armIO io = coralArm.getIO();
        return coralArm.runEnd(() -> io.setVoltage(voltage), () -> io.stopMotor())
                .withName("set voltage: " + voltage);
    }
}
