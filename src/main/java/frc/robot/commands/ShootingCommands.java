// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Factory class for high-level shooting / intaking sequences that coordinate Intake, Conveyor and
 * Shooter together.
 *
 * <h3>Sequence diagrams</h3>
 *
 * <pre>
 * INTAKE sequence (hold button):
 *   Intake rollers ON  ──────────────────────────────►  OFF on release
 *   Extension         EXTENDED ────────────────────►  (stays extended)
 *   Conveyor          BACKWARD (retain balls) ──────►  STOP on release
 *
 * SHOOT sequence (hold button):
 *   Flywheel          SPIN UP ─────────────────────────►  OFF on release
 *   Conveyor          ── wait spinup ──► FORWARD ────►  STOP on release
 *   Feeder            ── wait spinup ──► ON      ────►  OFF on release
 * </pre>
 */
public class ShootingCommands {

  private ShootingCommands() {}

  // ---------------------------------------------------------------------------
  // Intake + Conveyor coordination
  // ---------------------------------------------------------------------------

  /**
   * Runs the full intake sequence:
   *
   * <ul>
   *   <li>Extends the intake and runs the intake rollers to collect fuel (balls).
   *   <li>Simultaneously runs the conveyor <em>backward</em> to push balls toward the back of the
   *       robot so they don't fall out.
   * </ul>
   *
   * Stops both when the command ends (button released).
   */
  public static Command intakeWithRetainCommand(Intake intake, Conveyor conveyor) {
    return Commands.parallel(intake.runIntakeCommand(), conveyor.runRetainCommand());
  }

  /** Retracts the intake and stops the conveyor. */
  public static Command retractAndStopCommand(Intake intake, Conveyor conveyor) {
    return Commands.parallel(intake.retractCommand(), conveyor.stopCommand());
  }

  public static Command runShooter(Shooter shooter) {
    return Commands.runEnd(
        () -> {
          shooter.activateShoot(true);
          // shooter.activateIndexer(true);
        },
        () -> {
          shooter.stopShooterMotors();
        },
        shooter);
  }

  public static Command runIndexer(Shooter shooter) {
    return Commands.runEnd(
        () -> {
          shooter.activateIndexer(true);
        },
        () -> {
          shooter.stopIndexerMotors();
        },
        shooter);
  }

  public static Command shoot(Conveyor conveyor, Shooter shooter, Indexer indexer) {
    return Commands.parallel(
        shooter.runShooter(),
        new SequentialCommandGroup(
            new WaitCommand(0.4),
            new ParallelCommandGroup(conveyor.runConveyor(), indexer.runIndexer())));
  }

  // ---------------------------------------------------------------------------
  // Shoot sequence
  // ---------------------------------------------------------------------------

  /**
   * Full shoot sequence (hold-to-shoot):
   *
   * <ol>
   *   <li>Immediately starts spinning up the flywheel.
   *   <li>After {@value ShooterConstants#FLYWHEEL_SPINUP_SECONDS} s, enables:
   *       <ul>
   *         <li>the feeder (internal shooter motor),
   *         <li>the conveyor forward (pushes ball from conveyor into feeder).
   *       </ul>
   * </ol>
   *
   * All motors stop when the command is interrupted (button released).
   *
   * @param shooter the Shooter subsystem
   * @param conveyor the Conveyor subsystem
   */

  /*
  public static Command shootSequenceCommand(Shooter shooter, Conveyor conveyor) {
    return Commands.parallel(
        // 1. Start spinning the flywheel immediately`
        shooter.spinUpFlywheelCommand(),

        // 2. After spin-up delay: run feeder + conveyor together
        Commands.sequence(
            Commands.waitSeconds(ShooterConstants.FLYWHEEL_SPINUP_SECONDS),
            Commands.parallel(shooter.runFeederCommand(), conveyor.runForwardCommand())));
  }
            */

  /** Stops the entire shooter + conveyor system. */
  /*
  public static Command stopShooterSystem(Shooter shooter, Conveyor conveyor) {
    return Commands.parallel(shooter.stopCommand(), conveyor.stopCommand());
  }
    */
}
