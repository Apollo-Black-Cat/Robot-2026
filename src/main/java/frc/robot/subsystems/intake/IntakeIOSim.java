// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

  private final DCMotorSim extensionSim;
  private boolean extensionClosedLoop = false;
  private double extensionSetpointRotations = 0.0;
  private final PIDController extensionController =
      new PIDController(EXTENSION_KP, 0, EXTENSION_KD);
  private double extensionAppliedVolts = 0.0;

  public IntakeIOSim() {
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, ROLLER_MOI, ROLLER_GEAR_RATIO),
            ROLLER_GEARBOX);

    extensionSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EXTENSION_GEARBOX, EXTENSION_MOI, IntakeConstants.EXTENSION_GEAR_RATIO),
            EXTENSION_GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // ---- Extension closed-loop control ----
    if (extensionClosedLoop) {
      extensionAppliedVolts =
          extensionController.calculate(
              extensionSim.getAngularPositionRotations(), extensionSetpointRotations);
    } else {
      extensionController.reset();
    }

    // Step simulations
    rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
    extensionSim.setInputVoltage(MathUtil.clamp(extensionAppliedVolts, -12.0, 12.0));
    rollerSim.update(0.02);
    extensionSim.update(0.02);

    // ---- Roller inputs ----
    inputs.rollerConnected = true;
    inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());

    // ---- Extension inputs ----
    double extensionPosMeters =
        MathUtil.clamp(
            extensionSim.getAngularPositionRotations() * METERS_PER_ROTATION,
            0.0,
            IntakeConstants.MAX_EXTENSION_METERS);
    inputs.extensionConnected = true;
    inputs.extensionPositionMeters = extensionPosMeters;
    inputs.extensionVelocityMetersPerSec =
        extensionSim.getAngularVelocityRadPerSec() * METERS_PER_ROTATION / (2.0 * Math.PI);
    inputs.extensionAppliedVolts = extensionAppliedVolts;
    inputs.extensionCurrentAmps = Math.abs(extensionSim.getCurrentDrawAmps());
  }

  // ---- Roller ----

  @Override
  public void setRollerVoltage(double volts) {
    extensionClosedLoop = false;
    rollerAppliedVolts = volts;
  }

  // ---- Extension ----

  @Override
  public void setExtensionPosition(double positionMeters) {
    extensionClosedLoop = true;
    double clampedMeters =
        MathUtil.clamp(positionMeters, 0.0, IntakeConstants.MAX_EXTENSION_METERS);
    extensionSetpointRotations = clampedMeters / METERS_PER_ROTATION;
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionClosedLoop = false;
    extensionAppliedVolts = volts;
  }

  @Override
  public void resetExtensionEncoder() {
    // In sim we cannot truly reset; the position stays as-is.
    // If needed, set the sim state to 0 by providing zero initial conditions.
  }
}
