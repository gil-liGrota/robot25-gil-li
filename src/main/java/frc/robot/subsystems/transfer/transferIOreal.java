package frc.robot.subsystems.transfer;

import static frc.robot.subsystems.transfer.transferConstance.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class transferIOreal implements transferIO {

    private final POMDigitalInput transferSensor = new POMDigitalInput(TRANSFER_SENSOR_ID);
    private final POMSparkMax motor;
    private RelativeEncoder encoder;
    private final SparkMaxConfig config = new SparkMaxConfig();

    public transferIOreal() {
        motor = new POMSparkMax(TRANSFER_MOTOR_ID);
        encoder = motor.getEncoder();
        config.idleMode(IdleMode.kCoast);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

}