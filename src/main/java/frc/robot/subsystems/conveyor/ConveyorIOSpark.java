// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.conveyor;

import static frc.robot.util.PhoenixUtil.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import java.util.function.DoubleSupplier;

/** TalonFX implementation of the Conveyor IO. */
public class ConveyorIOSpark implements ConveyorIO {

  private final SparkMax motor;

  RelativeEncoder encoder;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  public ConveyorIOSpark() {
    SparkMaxConfig config = new SparkMaxConfig();
    motor = new SparkMax(ConveyorConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    config
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(Constants.ConveyorConstants.currentLimit)
        .voltageCompensation(12.0);
    config
        .encoder
        .velocityConversionFactor(2 * Math.PI / 60.0 / Constants.ConveyorConstants.motorReduction)
        .positionConversionFactor(2 * Math.PI / Constants.ConveyorConstants.motorReduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    // Apply config to motors
    config.inverted(Constants.ConveyorConstants.motorInverted);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder = motor.getEncoder();

    /*velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();*/
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    // get the velocity of the motors
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    // get the applied voltage and amps of the motors
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (value) -> inputs.appliedVolts = value[0] * value[1]);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getOutputCurrent, motor::getOutputCurrent},
        (value) -> inputs.currentAmps = value[0]);
  }

  @Override
  public void setVoltage() {
    motor.setVoltage(Constants.ConveyorConstants.maxVoltage);
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
