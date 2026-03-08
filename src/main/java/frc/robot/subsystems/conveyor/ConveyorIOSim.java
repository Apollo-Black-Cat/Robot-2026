// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation implementation of the Conveyor IO. */
public class ConveyorIOSim implements ConveyorIO {

  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double MOI = 0.002; // kg·m²
  private static final double GEAR_RATIO = 1.0;

  private final DCMotorSim sim;
  private double appliedVolts = 0.0;

  public ConveyorIOSim() {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEAR_RATIO), GEARBOX);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.motorConnected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
