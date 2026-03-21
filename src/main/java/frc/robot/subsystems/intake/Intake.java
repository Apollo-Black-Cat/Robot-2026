// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem – manages fuel collection (balls) for the robot.
 *
 * <h3>Mechanisms</h3>
 *
 * <ol>
 *   <li><b>Roller</b> – TalonFX spinning rollers that pull fuel (balls) in or push them out.
 *   <li><b>Extension (cremallera)</b> – TalonFX + rack-and-pinion that slides the intake
 *       horizontally between two preset positions:
 *       <ul>
 *         <li>{@link #RETRACTED_POSITION_METERS} – 0.0 m (fully in, stored inside the robot)
 *         <li>{@link #EXTENDED_POSITION_METERS} – {@value IntakeConstants#MAX_EXTENSION_METERS} m
 *             (fully deployed, ready to collect)
 *       </ul>
 * </ol>
 *
 * <h3>Usage (in RobotContainer)</h3>
 *
 * <pre>
 *   // Extend while right bumper held, retract on release:
 *   controller.rightBumper()
 *       .whileTrue(intake.runIntakeCommand())
 *       .onFalse(intake.retractCommand());
 * </pre>
 */
public class Intake extends SubsystemBase {

  /** Fully retracted position (intake stored inside robot frame). */
  public static final double RETRACTED_POSITION_METERS = 0.0;

  /** Fully extended position (intake deployed horizontally). */
  public static final double EXTENDED_POSITION_METERS = IntakeConstants.MAX_EXTENSION_METERS;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Desired extension position in meters. Updated by commands. */
  private double extensionGoalMeters = RETRACTED_POSITION_METERS;

  /** Whether the roller should be actively spinning. */
  private boolean rollerRunning = false;

  /** Voltage sent to the roller motor when spinning. Positive = intake. */
  private double rollerVolts = IntakeConstants.ROLLER_INTAKE_VOLTS;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Apply roller voltage
    // io.setRollerVoltage(rollerRunning ? rollerVolts : 0.0);

    // Apply extension goal (motor-side clamping is done in the IO layer)
    // io.setExtensionPosition(extensionGoalMeters);

    // Telemetry
    Logger.recordOutput("Intake/ExtensionGoalMeters", extensionGoalMeters);
    Logger.recordOutput("Intake/RollerRunning", rollerRunning);
    Logger.recordOutput("Intake/IsExtended", isExtended());
    Logger.recordOutput("Intake/IsRetracted", isRetracted());
  }

  // ---------------------------------------------------------------------------
  // State helpers
  // ---------------------------------------------------------------------------

  /** Returns {@code true} when the extension is within 5 mm of the fully extended position. */
  public boolean isExtended() {
    return Math.abs(inputs.extensionLeftPositionMeters - EXTENDED_POSITION_METERS) < 0.005;
  }

  /** Returns {@code true} when the extension is within 5 mm of the retracted position. */
  public boolean isRetracted() {
    return Math.abs(inputs.extensionLeftPositionMeters - RETRACTED_POSITION_METERS) < 0.005;
  }

  /** Returns the current extension position in meters. */
  public double getExtensionPositionMeters() {
    return inputs.extensionLeftPositionMeters;
  }

  /** Returns the current roller velocity in rad/s. */
  public double getRollerVelocityRadPerSec() {
    return inputs.rollerVelocityRadPerSec;
  }

  // ---------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------

  /**
   * Extends the intake to the fully deployed position and starts the rollers to collect fuel. When
   * interrupted, the rollers stop but the intake stays extended until retractCommand() is run.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          extensionGoalMeters = EXTENDED_POSITION_METERS;
          rollerRunning = true;
          rollerVolts = IntakeConstants.ROLLER_INTAKE_VOLTS;
        },
        () -> rollerRunning = false);
  }

  /**
   * Retracts the intake to the stored position (rollers off). Ends immediately after setting the
   * goal (MotionMagic handles the motion).
   */
  public Command retractCommand() {
    return this.runOnce(
        () -> {
          extensionGoalMeters = RETRACTED_POSITION_METERS;
          rollerRunning = false;
        });
  }

  /** Ejects / reverses the rollers to expel fuel. Does not change the extension position. */
  public Command ejectCommand() {
    return this.startEnd(
        () -> {
          rollerRunning = true;
          rollerVolts = IntakeConstants.ROLLER_EJECT_VOLTS;
        },
        () -> rollerRunning = false);
  }

  /**
   * Moves the extension to an arbitrary position (meters). The value is clamped to [0,
   * MAX_EXTENSION_METERS].
   */
  public Command setExtensionCommand(double positionMeters) {
    return this.runOnce(
        () -> {
          extensionGoalMeters =
              MathUtil.clamp(positionMeters, RETRACTED_POSITION_METERS, EXTENDED_POSITION_METERS);
        });
  }

  /** Manually drives the extension open-loop (for tuning / zeroing). Stops when interrupted. */
  public Command manualExtensionVoltageCommand(double volts) {
    return this.startEnd(() -> io.setExtensionVoltage(volts), () -> io.setExtensionVoltage(0.0));
  }

  public void extensionVoltage(double volts) {
    io.setExtensionVoltage(volts);
  }

  public void rollerVoltage(boolean isOn) {
    if (isOn) {
      io.setRollerVoltage(rollerVolts);
    } else {
      io.setRollerVoltage(-rollerVolts);
    }
  }

  public void stopRoller() {
    io.setRollerVoltage(0.0);
  }

  public void setDistance(double distance) {
    io.setExtensionPosition(distance);
  }

  public void stopMotors() {
    io.stopMotors();
  }

  public double getDistance() {
    return inputs.distance;
  }

  /** Zeros the extension encoder (call when mechanism is physically at the retracted hard stop). */
  public Command zeroExtensionCommand() {
    return this.runOnce(
        () -> {
          io.resetExtensionEncoder();
          extensionGoalMeters = RETRACTED_POSITION_METERS;
        });
  }

  public void roller(Boolean isOn) {
    if (isOn) {
      io.setRollerVoltage(rollerVolts);
    } else {
      io.setRollerVoltage(-rollerVolts);
    }
  }
}
