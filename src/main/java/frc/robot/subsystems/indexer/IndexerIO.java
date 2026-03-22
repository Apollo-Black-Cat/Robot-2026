// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

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
public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
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
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Run the feeder at the specified open-loop voltage (-12 to 12 V). */
  public default void setIndexerVoltage(double volts) {}

  public default void stopIndexerMotors() {}
}
