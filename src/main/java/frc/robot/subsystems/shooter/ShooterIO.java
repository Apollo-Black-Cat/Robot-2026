// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Shooter subsystem.
 *
 * <p>The shooter has two TalonFX motors:
 *
 * <ul>
 *   <li><b>Flywheel</b> – the final, high-speed wheel that actually launches the ball. Must be spun
 *       up (~0.5 s) before feeding.
 *   <li><b>Feeder</b> – an internal roller that accepts the ball from the conveyor and pushes it
 *       into the spinning flywheel once it is at speed.
 * </ul>
 */
public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    // ---- left indexer ----
    public boolean leftIndexerConnected = false;
    public double leftIndexerVelocityRadPerSec = 0.0;
    public double leftIndexerAppliedVolts = 0.0;
    public double leftIndexerCurrentAmps = 0.0;

    // ---- right indexer ----
    public boolean rightIndexerConnected = false;
    public double rightIndexerVelocityRadPerSec = 0.0;
    public double rightIndexerAppliedVolts = 0.0;
    public double rightIndexerCurrentAmps = 0.0;

    // ---- left shooter ----
    public boolean leftShooterConnected = false;
    public double leftShooterVelocityRadPerSec = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double leftShooterCurrentAmps = 0.0;

    // ---- right shooter ----
    public boolean rightShooterConnected = false;
    public double rightShooterVelocityRadPerSec = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double rightShooterCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the flywheel at the specified open-loop voltage (-12 to 12 V). */
  public default void setShooterVoltage(double volts) {}

  /** Run the feeder at the specified open-loop voltage (-12 to 12 V). */
  public default void setIndexerVoltage(double volts) {}

  public default void stopMotors() {}
}
