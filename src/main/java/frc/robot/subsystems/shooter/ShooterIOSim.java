// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation implementation of the Shooter IO. */
public class ShooterIOSim implements ShooterIO {

  private static final DCMotor LEFT_SHOOTER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor RIGHT_SHOOTER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private static final double SHOOTER_MOI = 0.008; // kg·m² – represents heavy flywheel
  private static final double INDEXER_MOI = 0.001; // kg·m²
  private static final double GEAR_RATIO = 1.0;

  private final DCMotorSim leftShooterSim;
  private final DCMotorSim rightShooterSim;
  private final DCMotorSim indexerSim;

  private double leftShooterAppliedVolts = 0.0;
  private double rightShooterAppliedVolts = 0.0;
  private double indexerAppliedVolts = 0.0;

  public ShooterIOSim() {
    leftShooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(LEFT_SHOOTER_GEARBOX, SHOOTER_MOI, GEAR_RATIO),
            LEFT_SHOOTER_GEARBOX);
    rightShooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(RIGHT_SHOOTER_GEARBOX, SHOOTER_MOI, GEAR_RATIO),
            RIGHT_SHOOTER_GEARBOX);
    indexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, INDEXER_MOI, GEAR_RATIO),
            INDEXER_GEARBOX);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    leftShooterSim.setInputVoltage(MathUtil.clamp(leftShooterAppliedVolts, -12.0, 12.0));
    rightShooterSim.setInputVoltage(MathUtil.clamp(leftShooterAppliedVolts, -12.0, 12.0));
    indexerSim.setInputVoltage(MathUtil.clamp(indexerAppliedVolts, -12.0, 12.0));
    leftShooterSim.update(0.02);
    rightShooterSim.update(0.02);
    indexerSim.update(0.02);

    inputs.leftShooterConnected = true;
    inputs.leftShooterVelocityRadPerSec = leftShooterSim.getAngularVelocityRadPerSec();
    inputs.leftShooterAppliedVolts = leftShooterAppliedVolts;
    inputs.leftShooterCurrentAmps = Math.abs(leftShooterSim.getCurrentDrawAmps());

    inputs.rightShooterConnected = true;
    inputs.rightShooterVelocityRadPerSec = rightShooterSim.getAngularVelocityRadPerSec();
    inputs.rightShooterAppliedVolts = rightShooterAppliedVolts;
    inputs.rightShooterCurrentAmps = Math.abs(rightShooterSim.getCurrentDrawAmps());

    inputs.leftIndexerConnected = true;
    inputs.leftIndexerVelocityRadPerSec = indexerSim.getAngularVelocityRadPerSec();
    inputs.leftIndexerAppliedVolts = indexerAppliedVolts;
    inputs.leftIndexerCurrentAmps = Math.abs(indexerSim.getCurrentDrawAmps());
  }

  @Override
  public void setShooterVoltage(double volts) {
    leftShooterAppliedVolts = volts;
    rightShooterAppliedVolts = volts;
  }

  @Override
  public void setIndexerVoltage(double volts) {
    indexerAppliedVolts = volts;
  }
}
