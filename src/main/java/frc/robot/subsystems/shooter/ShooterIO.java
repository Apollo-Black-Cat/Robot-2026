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
    // ---- Flywheel ----
    public boolean flywheelConnected = false;
    public double flywheelVelocityRadPerSec = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;

    // ---- Feeder ----
    public boolean feederConnected = false;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the flywheel at the specified open-loop voltage (-12 to 12 V). */
  public default void setFlywheelVoltage(double volts) {}

  /** Run the feeder at the specified open-loop voltage (-12 to 12 V). */
  public default void setFeederVoltage(double volts) {}
}
