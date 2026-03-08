// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/**
 * TalonFX implementation of the Intake IO.
 *
 * <p><b>Mechanical data (extension rack-and-pinion):</b>
 *
 * <ul>
 *   <li>Pinion pitch diameter: {@value IntakeConstants#PINION_DIAMETER_METERS} m (51.985 mm)
 *   <li>Pinion circumference: π × d ≈ 0.16327 m/revolution
 *   <li>Maximum travel: {@value IntakeConstants#MAX_EXTENSION_METERS} m (233.749 mm)
 *   <li>Gear ratio: 1:1 (motor shaft = pinion shaft; change in Constants if reduction exists)
 * </ul>
 */
public class IntakeIOTalonFX implements IntakeIO {

  // ---- Hardware ----
  private final TalonFX rollerTalon;
  private final TalonFX extensionTalon;

  // ---- Control requests ----
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut extensionVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVoltage extensionPositionRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(true);

  // ---- Status signals – roller ----
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerCurrent;

  // ---- Status signals – extension ----
  private final StatusSignal<Angle> extensionPosition;
  private final StatusSignal<AngularVelocity> extensionVelocity;
  private final StatusSignal<Voltage> extensionAppliedVolts;
  private final StatusSignal<Current> extensionCurrent;

  // ---- Connection debouncers ----
  private final Debouncer rollerConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer extensionConnectedDebouncer = new Debouncer(0.5);

  // ---- Conversion: rotations → meters for the rack ----
  // linear_m = rotations × circumference / gearRatio
  private static final double METERS_PER_ROTATION =
      Math.PI * IntakeConstants.PINION_DIAMETER_METERS / IntakeConstants.EXTENSION_GEAR_RATIO;

  public IntakeIOTalonFX() {
    rollerTalon = new TalonFX(IntakeConstants.ROLLER_CAN_ID, IntakeConstants.CAN_BUS);
    extensionTalon = new TalonFX(IntakeConstants.EXTENSION_CAN_ID, IntakeConstants.CAN_BUS);

    // ---- Configure roller ----
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerConfig, 0.25));

    // ---- Configure extension ----
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    extensionConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.EXTENSION_SUPPLY_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.EXTENSION_STATOR_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Software travel limits  (in rotations; 0 = retracted)
    double maxRotations = IntakeConstants.MAX_EXTENSION_METERS / METERS_PER_ROTATION;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotations;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    // PID + MotionMagic (gains to be tuned)
    extensionConfig.Slot0.kP = IntakeConstants.EXTENSION_KP;
    extensionConfig.Slot0.kI = IntakeConstants.EXTENSION_KI;
    extensionConfig.Slot0.kD = IntakeConstants.EXTENSION_KD;
    extensionConfig.Slot0.kS = IntakeConstants.EXTENSION_KS;
    extensionConfig.Slot0.kV = IntakeConstants.EXTENSION_KV;
    extensionConfig.Slot0.kA = IntakeConstants.EXTENSION_KA;
    extensionConfig.Slot0.kG = IntakeConstants.EXTENSION_KG;

    extensionConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.EXTENSION_MM_CRUISE_VELOCITY_RPS;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.EXTENSION_MM_ACCELERATION_RPS2;
    extensionConfig.MotionMagic.MotionMagicJerk = IntakeConstants.EXTENSION_MM_JERK_RPS3;

    tryUntilOk(5, () -> extensionTalon.getConfigurator().apply(extensionConfig, 0.25));

    // ---- Cache status signals ----
    rollerVelocity = rollerTalon.getVelocity();
    rollerAppliedVolts = rollerTalon.getMotorVoltage();
    rollerCurrent = rollerTalon.getSupplyCurrent();

    extensionPosition = extensionTalon.getPosition();
    extensionVelocity = extensionTalon.getVelocity();
    extensionAppliedVolts = extensionTalon.getMotorVoltage();
    extensionCurrent = extensionTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerVelocity,
        rollerAppliedVolts,
        rollerCurrent,
        extensionPosition,
        extensionVelocity,
        extensionAppliedVolts,
        extensionCurrent);

    rollerTalon.optimizeBusUtilization();
    extensionTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh all signals at once
    var rollerStatus =
        BaseStatusSignal.refreshAll(rollerVelocity, rollerAppliedVolts, rollerCurrent);
    var extensionStatus =
        BaseStatusSignal.refreshAll(
            extensionPosition, extensionVelocity, extensionAppliedVolts, extensionCurrent);

    inputs.rollerConnected = rollerConnectedDebouncer.calculate(rollerStatus.isOK());
    inputs.rollerVelocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();

    inputs.extensionConnected = extensionConnectedDebouncer.calculate(extensionStatus.isOK());
    inputs.extensionPositionMeters = extensionPosition.getValueAsDouble() * METERS_PER_ROTATION;
    inputs.extensionVelocityMetersPerSec =
        extensionVelocity.getValueAsDouble() * METERS_PER_ROTATION;
    inputs.extensionAppliedVolts = extensionAppliedVolts.getValueAsDouble();
    inputs.extensionCurrentAmps = extensionCurrent.getValueAsDouble();
  }

  // ---- Roller ----

  @Override
  public void setRollerVoltage(double volts) {
    rollerTalon.setControl(rollerVoltageRequest.withOutput(volts));
  }

  // ---- Extension ----

  @Override
  public void setExtensionPosition(double positionMeters) {
    double clampedMeters =
        edu.wpi.first.math.MathUtil.clamp(
            positionMeters, 0.0, IntakeConstants.MAX_EXTENSION_METERS);
    double rotations = clampedMeters / METERS_PER_ROTATION;
    extensionTalon.setControl(extensionPositionRequest.withPosition(rotations));
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionTalon.setControl(extensionVoltageRequest.withOutput(volts));
  }

  @Override
  public void resetExtensionEncoder() {
    tryUntilOk(5, () -> extensionTalon.setPosition(0.0, 0.25));
  }
}
