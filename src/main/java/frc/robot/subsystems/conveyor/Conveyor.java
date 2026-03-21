// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Conveyor subsystem.
 *
 * <p>A single TalonFX drives a belt that connects the Intake with the Shooter.
 *
 * <h3>Direction convention</h3>
 *
 * <ul>
 *   <li><b>Forward (+)</b> – toward the shooter (used when shooting or passing balls).
 *   <li><b>Backward (−)</b> – toward the intake (used while intaking to push balls back and keep
 *       them inside the robot frame).
 * </ul>
 */
public class Conveyor extends SubsystemBase {

  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  private double goalVolts = 0.0;

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);

    Logger.recordOutput("Conveyor/GoalVolts", goalVolts);
  }

  public void setVoltage() {
    io.setVoltage();
  }

  public void stopMotor() {
    io.stopMotor();
  }

  // ---------------------------------------------------------------------------
  // State helpers
  // ---------------------------------------------------------------------------

  /** Returns {@code true} when the conveyor is moving toward the shooter. */
  public boolean isRunningForward() {
    return goalVolts > 0;
  }

  /** Returns the current velocity in rad/s. */
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  // ---------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------

  /**
   * Runs the conveyor toward the shooter (positive voltage). Used to feed balls into the shooter.
   * Stops when interrupted.
   */
  public Command runForwardCommand() {
    return this.startEnd(() -> goalVolts = ConveyorConstants.FEED_VOLTS, () -> goalVolts = 0.0);
  }

  /**
   * Runs the conveyor backward (toward the intake) to retain balls inside the robot while
   * collecting. Stops when interrupted.
   */
  public Command runRetainCommand() {
    return this.startEnd(() -> goalVolts = ConveyorConstants.RETAIN_VOLTS, () -> goalVolts = 0.0);
  }

  /** Stops the conveyor immediately. */
  public Command stopCommand() {
    return this.runOnce(() -> goalVolts = 0.0);
  }
}
