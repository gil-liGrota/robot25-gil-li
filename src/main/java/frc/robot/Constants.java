// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class VisionConstants {
    // THE translations FOR THE LEFT AND RIGHT CAMERAS
    public static Translation3d l_camera_translation = new Translation3d(0.2 - 0.19 + 0.02 + 0.01 - 0.04,
        0.1 - 0.12 + 0.03 - 0.02, 0.263);
    public static Translation3d r_camera_translation = new Translation3d(0.03 + 0.02 + 0.02, -0.24 + 0.29 + 0.08 - 0.04,
        0.263);

    // THE ROTATION FOR THE LEFT AND RIGHT CAMERAS
    public static Rotation3d l_camera_rotation = new Rotation3d(Degrees.of(0), Degrees.of(0),
        Degrees.of(-15.7 - 5.5 - 1.7));
    public static Rotation3d r_camera_rotation = new Rotation3d(Degrees.of(0), Degrees.of(0),
        Degrees.of(19.9 - 13 + 5 + 10));

    // THE TRANSFORMATION FOR THE LEFT AND RIGHT CAM ERAS
    public static Transform3d l_camera_transform = new Transform3d(l_camera_translation, l_camera_rotation); // here
    public static Transform3d r_camera_transform = new Transform3d(r_camera_translation, r_camera_rotation);

    public static final Transform2d transformRightBranch = new Transform2d(0.384, 0.013, new Rotation2d(-0.1));
    public static final Transform2d transformLeftBranch = new Transform2d(0.384, 0.013, new Rotation2d(-0.1));
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
