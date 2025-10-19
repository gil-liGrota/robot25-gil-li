package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase {
    private armIO io;
    private armIOInputsAutoLogged inputs = new armIOInputsAutoLogged();

    public arm(armIO io) {
        this.io = io;
        setDefaultCommand(run(() -> io.stayInCurrentGoal()).withName("armDefaultCommand"));

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("arm", inputs);
    }

    public armIO getIO() {
        return io;
    }
}
