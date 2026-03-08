// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Conveyor subsystem.
 *
 * <p>A single TalonFX drives a belt/chain that moves fuel (balls) between the intake and the
 * shooter. Direction convention:
 *
 * <ul>
 *   <li><b>Forward (+)</b> – toward the shooter (feed mode).
 *   <li><b>Backward (−)</b> – toward the intake (retain mode, used while collecting so balls don't
 *       fall out).
 * </ul>
 */
public interface ConveyorIO {

  @AutoLog
  public static class ConveyorIOInputs {
    public boolean motorConnected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ConveyorIOInputs inputs) {}

  /** Run the conveyor at the specified open-loop voltage (-12 to 12 V). */
  public default void setVoltage(double volts) {}
}
