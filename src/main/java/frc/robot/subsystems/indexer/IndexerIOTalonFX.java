// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

/** TalonFX implementation of the Shooter IO (flywheel + feeder). */
public class IndexerIOTalonFX implements IndexerIO {

  // ---- Hardware ----
  private final TalonFX leftShooterTalon;
  private final TalonFX rightShooterTalon;
  private final TalonFX leftIndexerTalon;
  private final TalonFX rightIndexerTalon;

  // ---- Control requests ----
  private final VoltageOut indexerVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  // private final VoltageOut rightShooterVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  // ---- Status signals – indexer ----
  private final StatusSignal<AngularVelocity> leftIndexerVelocity;
  private final StatusSignal<Voltage> leftIndexerAppliedVolts;
  private final StatusSignal<Current> leftIndexerCurrent;

  private final StatusSignal<AngularVelocity> rightIndexerVelocity;
  private final StatusSignal<Voltage> rightIndexerAppliedVolts;
  private final StatusSignal<Current> rightIndexerCurrent;

  // ---- Status signals – leftShooter ----
  private final StatusSignal<AngularVelocity> leftShooterVelocity;
  private final StatusSignal<Voltage> leftShooterAppliedVolts;
  private final StatusSignal<Current> leftShooterCurrent;

  // ---- Status signals – rightShooter ----
  private final StatusSignal<AngularVelocity> rightShooterVelocity;
  private final StatusSignal<Voltage> rightShooterAppliedVolts;
  private final StatusSignal<Current> rightShooterCurrent;

  // ---- Debouncers ----
  private final Debouncer leftIndexerConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer rightIndexerConnectedDebouncer = new Debouncer(0.5);

  public IndexerIOTalonFX() {
    leftIndexerTalon = new TalonFX(ShooterConstants.LEFTINDEXER_CAN_ID);
    rightIndexerTalon = new TalonFX(ShooterConstants.RIGHTINDEXER_CAN_ID);
    leftShooterTalon = new TalonFX(ShooterConstants.LEFTSHOOTER_CAN_ID);
    rightShooterTalon = new TalonFX(ShooterConstants.RIGHTSHOOTER_CAN_ID);

    // ---- Flywheel config ----
    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    tryUntilOk(5, () -> leftIndexerTalon.getConfigurator().apply(indexerConfig, 0.25));
    indexerConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    tryUntilOk(5, () -> rightIndexerTalon.getConfigurator().apply(indexerConfig, 0.25));

    // ---- Feeder config ----
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FEEDER_SUPPLY_CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FEEDER_STATOR_CURRENT_LIMIT;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    tryUntilOk(5, () -> leftShooterTalon.getConfigurator().apply(shooterConfig, 0.25));
    shooterConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    tryUntilOk(5, () -> rightShooterTalon.getConfigurator().apply(shooterConfig, 0.25));

    // ---- Cache signals ----
    leftIndexerVelocity = leftIndexerTalon.getVelocity();
    leftIndexerAppliedVolts = leftIndexerTalon.getMotorVoltage();
    leftIndexerCurrent = leftIndexerTalon.getSupplyCurrent();

    rightIndexerVelocity = rightIndexerTalon.getVelocity();
    rightIndexerAppliedVolts = rightIndexerTalon.getMotorVoltage();
    rightIndexerCurrent = rightIndexerTalon.getSupplyCurrent();

    leftShooterVelocity = leftShooterTalon.getVelocity();
    leftShooterAppliedVolts = leftShooterTalon.getMotorVoltage();
    leftShooterCurrent = leftShooterTalon.getSupplyCurrent();

    rightShooterVelocity = rightShooterTalon.getVelocity();
    rightShooterAppliedVolts = rightShooterTalon.getMotorVoltage();
    rightShooterCurrent = rightShooterTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftIndexerVelocity,
        leftIndexerAppliedVolts,
        leftIndexerCurrent,
        rightIndexerVelocity,
        rightIndexerAppliedVolts,
        rightIndexerCurrent,
        leftShooterVelocity,
        leftShooterAppliedVolts,
        leftShooterCurrent,
        rightShooterVelocity,
        rightShooterAppliedVolts,
        rightShooterCurrent);

    leftIndexerTalon.optimizeBusUtilization();
    rightIndexerTalon.optimizeBusUtilization();
    leftShooterTalon.optimizeBusUtilization();
    rightShooterTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var rightIndexerStatus =
        BaseStatusSignal.refreshAll(
            rightIndexerVelocity, rightIndexerAppliedVolts, rightIndexerCurrent);
    var leftIndexerStatus =
        BaseStatusSignal.refreshAll(
            leftIndexerVelocity, leftIndexerAppliedVolts, leftIndexerCurrent);

    inputs.leftIndexerConnected = leftIndexerConnectedDebouncer.calculate(leftIndexerStatus.isOK());
    inputs.leftIndexerVelocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(leftIndexerVelocity.getValueAsDouble());
    inputs.leftIndexerAppliedVolts = leftIndexerAppliedVolts.getValueAsDouble();
    inputs.leftIndexerCurrentAmps = leftIndexerCurrent.getValueAsDouble();

    inputs.rightIndexerConnected =
        rightIndexerConnectedDebouncer.calculate(rightIndexerStatus.isOK());
    inputs.rightIndexerVelocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(rightIndexerVelocity.getValueAsDouble());
    inputs.rightIndexerAppliedVolts = rightIndexerAppliedVolts.getValueAsDouble();
    inputs.rightIndexerCurrentAmps = rightIndexerCurrent.getValueAsDouble();
  }

  @Override
  public void setIndexerVoltage(double volts) {
    leftIndexerTalon.setControl(indexerVoltageRequest.withOutput(volts));
    rightIndexerTalon.setControl(leftIndexerTalon.getAppliedControl());
  }

  @Override
  public void stopIndexerMotors() {
    leftIndexerTalon.stopMotor();
    rightIndexerTalon.stopMotor();
  }
}
