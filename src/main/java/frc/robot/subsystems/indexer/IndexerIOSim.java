// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation implementation of the Shooter IO. */
public class IndexerIOSim implements IndexerIO {

  private static final DCMotor INDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private static final double INDEXER_MOI = 0.001; // kg·m²
  private static final double GEAR_RATIO = 1.0;

  private final DCMotorSim indexerSim;

  private double indexerAppliedVolts = 0.0;

  public IndexerIOSim() {
    indexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_GEARBOX, INDEXER_MOI, GEAR_RATIO),
            INDEXER_GEARBOX);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerSim.setInputVoltage(MathUtil.clamp(indexerAppliedVolts, -12.0, 12.0));
    indexerSim.update(0.02);

    inputs.leftIndexerConnected = true;
    inputs.leftIndexerVelocityRadPerSec = indexerSim.getAngularVelocityRadPerSec();
    inputs.leftIndexerAppliedVolts = indexerAppliedVolts;
    inputs.leftIndexerCurrentAmps = Math.abs(indexerSim.getCurrentDrawAmps());
  }

  @Override
  public void setIndexerVoltage(double volts) {
    indexerAppliedVolts = volts;
  }
}
