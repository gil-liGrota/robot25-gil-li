package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class transfer extends SubsystemBase {

    private final transferIO io;
    private final transferIOInputsAutoLogged transferInputs = new transferIOInputsAutoLogged();

    public transfer(transferIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(transferInputs);
        Logger.processInputs("transfer", transferInputs);
    }

    public transferIO getIO() {
        return io;
    }
}
