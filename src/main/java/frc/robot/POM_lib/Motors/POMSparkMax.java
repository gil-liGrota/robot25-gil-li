package frc.robot.POM_lib.Motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class POMSparkMax extends SparkMax implements POMMotor {
  public POMSparkMax(int id) {
    this(id, MotorType.kBrushless);
  }

  public POMSparkMax(int id, MotorType type) {
    super(id, type);
  }

  @Override
  public void stop() {
    set(0);
  }

  @Override
  public void setDirection(Direction direction) {

    setDirection(direction);
  }

  @Override
  public void setBrake(boolean isBrake) {
    setBrake(isBrake);
  }

  // public void configure(SparkMaxConfig config, ResetMode
  // knoresetsafeparameters,
  // PersistMode kpersistparameters) {
  // // TODO Auto-generated method stub
  // throw new UnsupportedOperationException("Unimplemented method 'configure'");
  // }
}
