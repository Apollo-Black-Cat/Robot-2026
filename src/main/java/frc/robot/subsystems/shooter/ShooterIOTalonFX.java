// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

/** TalonFX implementation of the Shooter IO (flywheel + feeder). */
public class ShooterIOTalonFX implements ShooterIO {

  // ---- Hardware ----
  private final TalonFX flywheelTalon;
  private final TalonFX feederTalon;

  // ---- Control requests ----
  private final VoltageOut flywheelVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut feederVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  // ---- Status signals – flywheel ----
  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;

  // ---- Status signals – feeder ----
  private final StatusSignal<AngularVelocity> feederVelocity;
  private final StatusSignal<Voltage> feederAppliedVolts;
  private final StatusSignal<Current> feederCurrent;

  // ---- Debouncers ----
  private final Debouncer flywheelConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer feederConnectedDebouncer = new Debouncer(0.5);

  public ShooterIOTalonFX() {
    flywheelTalon = new TalonFX(ShooterConstants.FLYWHEEL_CAN_ID, ShooterConstants.CAN_BUS);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_CAN_ID, ShooterConstants.CAN_BUS);

    // ---- Flywheel config ----
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> flywheelTalon.getConfigurator().apply(flywheelConfig, 0.25));

    // ---- Feeder config ----
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FEEDER_SUPPLY_CURRENT_LIMIT;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FEEDER_STATOR_CURRENT_LIMIT;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> feederTalon.getConfigurator().apply(feederConfig, 0.25));

    // ---- Cache signals ----
    flywheelVelocity = flywheelTalon.getVelocity();
    flywheelAppliedVolts = flywheelTalon.getMotorVoltage();
    flywheelCurrent = flywheelTalon.getSupplyCurrent();

    feederVelocity = feederTalon.getVelocity();
    feederAppliedVolts = feederTalon.getMotorVoltage();
    feederCurrent = feederTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelVelocity,
        flywheelAppliedVolts,
        flywheelCurrent,
        feederVelocity,
        feederAppliedVolts,
        feederCurrent);

    flywheelTalon.optimizeBusUtilization();
    feederTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var flywheelStatus =
        BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAppliedVolts, flywheelCurrent);
    var feederStatus =
        BaseStatusSignal.refreshAll(feederVelocity, feederAppliedVolts, feederCurrent);

    inputs.flywheelConnected = flywheelConnectedDebouncer.calculate(flywheelStatus.isOK());
    inputs.flywheelVelocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(flywheelVelocity.getValueAsDouble());
    inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValueAsDouble();
    inputs.flywheelCurrentAmps = flywheelCurrent.getValueAsDouble();

    inputs.feederConnected = feederConnectedDebouncer.calculate(feederStatus.isOK());
    inputs.feederVelocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(feederVelocity.getValueAsDouble());
    inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
    inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelTalon.setControl(flywheelVoltageRequest.withOutput(volts));
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederTalon.setControl(feederVoltageRequest.withOutput(volts));
  }
}
