// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.conveyor;

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
import frc.robot.Constants.ConveyorConstants;

/** TalonFX implementation of the Conveyor IO. */
public class ConveyorIOTalonFX implements ConveyorIO {

  private final TalonFX motor;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ConveyorIOTalonFX() {
    motor = new TalonFX(ConveyorConstants.MOTOR_CAN_ID, ConveyorConstants.CAN_BUS);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = ConveyorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ConveyorConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);
    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.velocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
