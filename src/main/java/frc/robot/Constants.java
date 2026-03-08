// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Constants for the Intake subsystem. */
  public static final class IntakeConstants {
    // ---- CAN IDs ----
    public static final int ROLLER_CAN_ID = 20;
    public static final int EXTENSION_CAN_ID = 21;

    /** CAN bus name. Use "" or "rio" for the roboRIO bus, or "CANivore" if using a CANivore. */
    public static final String CAN_BUS = "rio";

    // ---- Roller current limits ----
    public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0; // amps
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 80.0; // amps

    // ---- Extension current limits ----
    public static final double EXTENSION_SUPPLY_CURRENT_LIMIT = 30.0; // amps
    public static final double EXTENSION_STATOR_CURRENT_LIMIT = 60.0; // amps

    // ---- Rack-and-pinion geometry ----
    /** Pinion pitch diameter in meters (51.985 mm). */
    public static final double PINION_DIAMETER_METERS = 0.051985;

    /**
     * Gear ratio between the TalonFX output shaft and the pinion. Set to 1.0 if the motor drives
     * the pinion directly (1:1). Increase if there is an additional reduction stage.
     */
    public static final double EXTENSION_GEAR_RATIO = 1.0;

    /** Maximum allowable extension travel in meters (233.749 mm). */
    public static final double MAX_EXTENSION_METERS = 0.233749;

    // ---- Extension PID / feedforward gains (to be tuned in robot) ----
    public static final double EXTENSION_KP = 40.0;
    public static final double EXTENSION_KI = 0.0;
    public static final double EXTENSION_KD = 0.5;
    public static final double EXTENSION_KS = 0.25; // volts (static friction)
    public static final double EXTENSION_KV = 0.12; // volts·s/rot
    public static final double EXTENSION_KA = 0.01; // volts·s²/rot
    public static final double EXTENSION_KG = 0.0; // volts (gravity; 0 for horizontal)

    // ---- MotionMagic constraints (in rotations/s and rotations/s²) ----
    /** Cruise velocity in rotations per second. */
    public static final double EXTENSION_MM_CRUISE_VELOCITY_RPS = 10.0;
    /** Acceleration in rotations per second². */
    public static final double EXTENSION_MM_ACCELERATION_RPS2 = 20.0;
    /** Jerk in rotations per second³ (0 = disabled). */
    public static final double EXTENSION_MM_JERK_RPS3 = 0.0;

    // ---- Roller preset voltages ----
    public static final double ROLLER_INTAKE_VOLTS = 10.0;
    public static final double ROLLER_EJECT_VOLTS = -8.0;
  }

  /** Constants for the Conveyor subsystem. */
  public static final class ConveyorConstants {
    // ---- CAN IDs ----
    public static final int MOTOR_CAN_ID = 30;

    /** CAN bus name. Use "rio" for the roboRIO bus, or "CANivore" if using a CANivore. */
    public static final String CAN_BUS = "rio";

    // ---- Current limits ----
    public static final double SUPPLY_CURRENT_LIMIT = 30.0; // amps
    public static final double STATOR_CURRENT_LIMIT = 60.0; // amps

    // ---- Preset voltages ----
    /** Voltage to run the conveyor toward the shooter (feeding/shooting). */
    public static final double FEED_VOLTS = 9.0;
    /**
     * Voltage to run the conveyor backward (toward the intake) while collecting, to prevent balls
     * from falling out. Negative = backward.
     */
    public static final double RETAIN_VOLTS = -4.0;
  }

  /** Constants for the Shooter subsystem. */
  public static final class ShooterConstants {
    // ---- CAN IDs ----
    public static final int FLYWHEEL_CAN_ID = 31;
    public static final int FEEDER_CAN_ID = 32;

    /** CAN bus name. */
    public static final String CAN_BUS = "rio";

    // ---- Flywheel current limits ----
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 40.0; // amps
    public static final double FLYWHEEL_STATOR_CURRENT_LIMIT = 80.0; // amps

    // ---- Feeder current limits ----
    public static final double FEEDER_SUPPLY_CURRENT_LIMIT = 30.0; // amps
    public static final double FEEDER_STATOR_CURRENT_LIMIT = 60.0; // amps

    // ---- Preset voltages ----
    /** Voltage applied to the flywheel motor during a shot. */
    public static final double FLYWHEEL_SHOOT_VOLTS = 11.0;
    /** Voltage applied to the feeder motor when feeding a ball into the flywheel. */
    public static final double FEEDER_SHOOT_VOLTS = 8.0;

    // ---- Spin-up ----
    /**
     * Minimum time (seconds) to wait after the flywheel motor is commanded before enabling the
     * feeder. This gives the flywheel enough time to reach shooting speed.
     */
    public static final double FLYWHEEL_SPINUP_SECONDS = 0.5;

    /**
     * Flywheel target speed in rad/s used to determine "at speed". Corresponds roughly to
     * FLYWHEEL_SHOOT_VOLTS at free-speed for a Kraken X60. Tune after measuring actual free-speed.
     */
    public static final double FLYWHEEL_TARGET_RAD_PER_SEC = 500.0; // ~4775 RPM

    /**
     * Tolerance as a fraction (0–1). The flywheel is considered "at speed" when its velocity is ≥
     * {@code FLYWHEEL_TARGET_RAD_PER_SEC × (1 − FLYWHEEL_TOLERANCE_PERCENT)}.
     */
    public static final double FLYWHEEL_TOLERANCE_PERCENT = 0.05; // 5 %
  }
}
