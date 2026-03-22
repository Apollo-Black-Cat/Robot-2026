// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem.
 *
 * <h3>Mechanism overview</h3>
 *
 * <ol>
 *   <li><b>Flywheel</b> – high-inertia launch wheel. Must reach target speed before a ball can be
 *       fed, otherwise the shot is inconsistent. Spin-up takes ~ {@value
 *       ShooterConstants#FLYWHEEL_SPINUP_SECONDS} s.
 *   <li><b>Feeder</b> – internal roller that accepts the ball from the conveyor and pushes it into
 *       the spinning flywheel. Only active once the flywheel is at speed.
 * </ol>
 *
 * <h3>State machine (managed per-command)</h3>
 *
 * <pre>
 *   IDLE  →  spinUpFlywheelCommand()  →  SPINNING
 *   SPINNING  →  feedCommand()        →  FEEDING  (feeder on + flywheel stays on)
 *   any state  →  stopCommand()       →  IDLE
 * </pre>
 *
 * <p>The {@code shootCommand()} convenience method wires the full sequence together.
 */
public class Indexer extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private double indexerGoalVolts = 0.0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/FeederGoalVolts", indexerGoalVolts);
  }

  public void activateIndexer(Boolean isOn) {
    io.setIndexerVoltage(Constants.ShooterConstants.FEEDER_SHOOT_VOLTS);
  }

  public void stopIndexerMotors() {
    io.stopIndexerMotors();
  }

  public Command runIndexer() {
    return run(() -> {
          io.setIndexerVoltage(Constants.ShooterConstants.FEEDER_SHOOT_VOLTS);
        })
        .handleInterrupt(
            () -> {
              io.stopIndexerMotors();
            })
        .finallyDo(
            () -> {
              io.stopIndexerMotors();
            });
  }

  // ---------------------------------------------------------------------------
  // State helpers
  // ---------------------------------------------------------------------------

  /**
   * Returns {@code true} when the flywheel velocity is within the configured tolerance of the
   * target speed for shooting.
   */
  // ---------------------------------------------------------------------------
  // Commands
  // ---------------------------------------------------------------------------

  /**
   * Spins up only the flywheel to shooting speed. The feeder stays OFF. Use this while waiting for
   * the robot to align before shooting. Stops when interrupted.
   */
  /*
  public Command spinUpFlywheelCommand() {
    return this.startEnd(
        () -> {
          leftShooterGoalVolts = ShooterConstants.FLYWHEEL_SHOOT_VOLTS;
          feederGoalVolts = 0.0;
        },
        () -> {
          flywheelGoalVolts = 0.0;
          feederGoalVolts = 0.0;
        });
  }
         */

  /**
   * Runs the feeder (internal roller) at feeding voltage. Intended to be used <em>after</em> the
   * flywheel is already at speed. Does NOT start the flywheel — chain with {@link
   * #spinUpFlywheelCommand()} or {@link #shootCommand()} for the full sequence. Stops when
   * interrupted.
   */
  /*
  public Command runFeederCommand() {
    return this.startEnd(
        () -> feederGoalVolts = ShooterConstants.FEEDER_SHOOT_VOLTS, () -> feederGoalVolts = 0.0);
  }
        */

  /**
   * Full shoot sequence (convenience command):
   *
   * <ol>
   *   <li>Spin up the flywheel ({@value ShooterConstants#FLYWHEEL_SPINUP_SECONDS} s delay).
   *   <li>Start the feeder once spin-up time has elapsed.
   * </ol>
   *
   * Both motors keep running until the command is interrupted.
   */

  /*
  public Command shootCommand() {
    return this.startEnd(
            () -> flywheelGoalVolts = ShooterConstants.FLYWHEEL_SHOOT_VOLTS,
            () -> {
              flywheelGoalVolts = 0.0;
              feederGoalVolts = 0.0;
            })
        // After FLYWHEEL_SPINUP_SECONDS, also enable the feeder
        .alongWith(
            edu.wpi.first.wpilibj2.command.Commands.waitSeconds(
                    ShooterConstants.FLYWHEEL_SPINUP_SECONDS)
                .andThen(
                    edu.wpi.first.wpilibj2.command.Commands.runOnce(
                        () -> feederGoalVolts = ShooterConstants.FEEDER_SHOOT_VOLTS)));
  }
                        */

  /** Stops both flywheel and feeder immediately. */
  /*
  public Command stopCommand() {
    return this.runOnce(
        () -> {
          flywheelGoalVolts = 0.0;
          feederGoalVolts = 0.0;
        });
  }
        */
}
