package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {

    private elevatorIO io;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    public elevator(elevatorIO io) {
        this.io = io;
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
