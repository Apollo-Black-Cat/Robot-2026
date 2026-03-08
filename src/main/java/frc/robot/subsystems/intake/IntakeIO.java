// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Intake subsystem.
 *
 * <p>The intake has two independent mechanisms:
 *
 * <ul>
 *   <li><b>Roller (rodamientos)</b> – TalonFX that spins to collect/eject fuel (balls).
 *   <li><b>Extension (cremallera)</b> – TalonFX driving a rack-and-pinion with a 51.985 mm pitch
 *       diameter gear, maximum travel 233.749 mm.
 * </ul>
 */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    // ---------- Roller ----------
    public boolean rollerConnected = false;
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;

    // ---------- Extension ----------
    public boolean extensionConnected = false;
    /** Current extension position in meters (0 = fully retracted). */
    public double extensionPositionMeters = 0.0;

    public double extensionVelocityMetersPerSec = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  // -------------------------------------------------------------------------
  // Roller control
  // -------------------------------------------------------------------------

  /** Run the roller at the specified open-loop voltage (-12 to 12 V). */
  public default void setRollerVoltage(double volts) {}

  // -------------------------------------------------------------------------
  // Extension control
  // -------------------------------------------------------------------------

  /**
   * Command the extension to a specific position (meters, 0 = retracted). Implementations should
   * enforce the 233.749 mm travel limit in hardware.
   */
  public default void setExtensionPosition(double positionMeters) {}

  /** Run the extension motor at the specified open-loop voltage (-12 to 12 V). */
  public default void setExtensionVoltage(double volts) {}

  /** Zero the extension encoder (call when the mechanism is known to be at the retracted limit). */
  public default void resetExtensionEncoder() {}
}
