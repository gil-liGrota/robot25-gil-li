package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {

    private elevatorIO io;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    public elevator(elevatorIO io) {
        this.io = io;
        setDefaultCommand(new RepeatCommand(new ConditionalCommand(this.runOnce(() -> io.setVoltage(0)),
                this.runOnce(io::resistGravity), io::isPressed))
                .beforeStarting(new PrintCommand("Elevator default command")));
    }

    @Override
    public void periodic() {
        io.updateInputs(elevatorInputs);
        Logger.processInputs("elevator", elevatorInputs);
    }

    public elevatorIO getIO() {
        return io;
    }
}
