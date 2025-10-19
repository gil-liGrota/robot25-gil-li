package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class armConstance {
    public static final int CORAL_ARM_ID = 17;
    public static final int HIGH_SWITCH = 1;
    public static final int LOW_SWITCH = 2;
    public static final int BRAKE_SWITCH = 4;

    public static final double KP = 3.4;
    public static final double KI = 0.0;
    public static final double KD = 0.005;
    public static final double KS = 0.35;
    public static final double KG = 0.45;
    public static final double KV = 0;

    public static final double MAX_ACCELERATION = 10.0;
    public static final double MAX_VELOCITY = 10.0;

    public static final double RESIST_GRAVITY = 0;
    public static final double TOLERANCE = Units.degreesToRadians(3);

    public static final double FORWARD_SOFT_LIMIT = 0;
    public static final double L1_ARM_POSITION = -1.24;
    public static final double L2_ARM_POSITION = -1.2;
    public static final double L3_ARM_POSITION = 1.1;
    public static final double L4_ARM_POSITION = 1.28;
    public static final double OPEN_ARM_POSITION = Math.PI / 2;
    public static final double CLOSE_ARM_POSITION = -Math.PI / 2;

    public static final double POSITION_CONVERSION_FACTOR = 1 / 40.0 /* versa */ * (2 * Math.PI) /* to radians */; // TODO
                                                                                                                   // verify

    public static double KG_OF_CORAL = 0;

    public static final boolean INVERTED = true;

    public static final int CURRENT_LIMIT = 40;

    public static final double VOLTAGE_COMPENSATION = 12.0;

}
