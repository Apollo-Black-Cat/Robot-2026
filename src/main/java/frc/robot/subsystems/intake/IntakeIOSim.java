// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

/**
 * Physics simulation implementation of the Intake IO.
 *
 * <p>Models both the roller (spinning mass) and the extension (rack-and-pinion linear actuator)
 * using WPILib DCMotorSim. All simulation control is voltage-based.
 */
public class IntakeIOSim implements IntakeIO {

  // ---- Roller sim ----
  private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double ROLLER_MOI = 0.001; // kg·m² – tune as needed
  private static final double ROLLER_GEAR_RATIO = 1.0;

  private final DCMotorSim rollerSim;
  private double rollerAppliedVolts = 0.0;

  // ---- Extension sim ----
  private static final DCMotor EXTENSION_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double EXTENSION_MOI = 0.005; // kg·m²
  private static final double EXTENSION_KP = 40.0;
  private static final double EXTENSION_KD = 0.5;

  // linear_m = rotations × circumference / gearRatio
  private static final double METERS_PER_ROTATION =
      Math.PI * IntakeConstants.PINION_DIAMETER_METERS / IntakeConstants.EXTENSION_GEAR_RATIO;

  private final ProfiledPIDController m_conController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(1.5, 1.0));

  private final DCMotorSim extensionLeftSim;
  private boolean extensionLeftClosedLoop = false;
  private double extensionLeftSetpointRotations = 0.0;
  private final PIDController extensionLeftController =
      new PIDController(EXTENSION_KP, 0, EXTENSION_KD);
  private double extensionLeftAppliedVolts = 0.0;

  private final DCMotorSim extensionRightSim;
  private boolean extensionRightClosedLoop = false;
  private double extensionRightSetpointRotations = 0.0;
  private final PIDController extensionRightController =
      new PIDController(EXTENSION_KP, 0, EXTENSION_KD);
  private double extensionRightAppliedVolts = 0.0;

  public IntakeIOSim() {
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, ROLLER_MOI, ROLLER_GEAR_RATIO),
            ROLLER_GEARBOX);

    extensionLeftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EXTENSION_GEARBOX, EXTENSION_MOI, IntakeConstants.EXTENSION_GEAR_RATIO),
            EXTENSION_GEARBOX);

    extensionRightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EXTENSION_GEARBOX, EXTENSION_MOI, IntakeConstants.EXTENSION_GEAR_RATIO),
            EXTENSION_GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // ---- Extension closed-loop control ----
    if (extensionLeftClosedLoop) {
      extensionLeftAppliedVolts =
          extensionLeftController.calculate(
              extensionLeftSim.getAngularPositionRotations(), extensionLeftSetpointRotations);
    } else {
      extensionLeftController.reset();
    }

    // Step simulations
    rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
    extensionLeftSim.setInputVoltage(MathUtil.clamp(extensionLeftAppliedVolts, -12.0, 12.0));
    extensionRightSim.setInputVoltage(MathUtil.clamp(extensionRightAppliedVolts, -12.0, 12.0));
    rollerSim.update(0.02);
    extensionLeftSim.update(0.02);
    extensionRightSim.update(0.02);

    // ---- Roller inputs ----
    inputs.rollerConnected = true;
    inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());

    // ---- Extension inputs ----
    double extensionPosMeters =
        MathUtil.clamp(
            extensionLeftSim.getAngularPositionRotations() * METERS_PER_ROTATION,
            0.0,
            IntakeConstants.MAX_EXTENSION_METERS);
    inputs.extensionLeftConnected = true;
    inputs.extensionLeftPositionMeters = extensionPosMeters;
    inputs.extensionLeftVelocityMetersPerSec =
        extensionLeftSim.getAngularVelocityRadPerSec() * METERS_PER_ROTATION / (2.0 * Math.PI);
    inputs.extensionLeftAppliedVolts = extensionLeftAppliedVolts;
    inputs.extensionLeftCurrentAmps = Math.abs(extensionLeftSim.getCurrentDrawAmps());

    inputs.extensionRightConnected = true;
    inputs.extensionRightPositionMeters = extensionPosMeters;
    inputs.extensionRightVelocityMetersPerSec =
        extensionLeftSim.getAngularVelocityRadPerSec() * METERS_PER_ROTATION / (2.0 * Math.PI);
    inputs.extensionRightAppliedVolts = extensionLeftAppliedVolts;
    inputs.extensionRightCurrentAmps = Math.abs(extensionLeftSim.getCurrentDrawAmps());
  }

  // ---- Roller ----

  @Override
  public void setRollerVoltage(double volts) {
    extensionLeftClosedLoop = false;
    extensionRightClosedLoop = false;
    rollerAppliedVolts = volts;
  }

  // ---- Extension ----

  @Override
  public void setExtensionPosition(double positionMeters) {
    extensionLeftClosedLoop = true;
    extensionRightClosedLoop = true;
    double clampedLeftMeters =
        MathUtil.clamp(positionMeters, 0.0, IntakeConstants.MAX_EXTENSION_METERS);
    extensionLeftSetpointRotations = clampedLeftMeters / METERS_PER_ROTATION;

    double clampedRightMeters =
        MathUtil.clamp(positionMeters, 0.0, IntakeConstants.MAX_EXTENSION_METERS);
    extensionRightSetpointRotations = clampedRightMeters / METERS_PER_ROTATION;
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionLeftClosedLoop = false;
    extensionRightClosedLoop = false;
    extensionLeftAppliedVolts = volts;
    extensionRightAppliedVolts = volts;
  }

  @Override
  public void resetExtensionEncoder() {
    // In sim we cannot truly reset; the position stays as-is.
    // If needed, set the sim state to 0 by providing zero initial conditions.
  }
}
