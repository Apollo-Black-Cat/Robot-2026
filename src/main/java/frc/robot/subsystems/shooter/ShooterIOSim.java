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

  private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor FEEDER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private static final double FLYWHEEL_MOI = 0.008; // kg·m² – represents heavy flywheel
  private static final double FEEDER_MOI = 0.001; // kg·m²
  private static final double GEAR_RATIO = 1.0;

  private final DCMotorSim flywheelSim;
  private final DCMotorSim feederSim;

  private double flywheelAppliedVolts = 0.0;
  private double feederAppliedVolts = 0.0;

  public ShooterIOSim() {
    flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(FLYWHEEL_GEARBOX, FLYWHEEL_MOI, GEAR_RATIO),
            FLYWHEEL_GEARBOX);
    feederSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(FEEDER_GEARBOX, FEEDER_MOI, GEAR_RATIO),
            FEEDER_GEARBOX);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.setInputVoltage(MathUtil.clamp(flywheelAppliedVolts, -12.0, 12.0));
    feederSim.setInputVoltage(MathUtil.clamp(feederAppliedVolts, -12.0, 12.0));
    flywheelSim.update(0.02);
    feederSim.update(0.02);

    inputs.flywheelConnected = true;
    inputs.flywheelVelocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheelCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());

    inputs.feederConnected = true;
    inputs.feederVelocityRadPerSec = feederSim.getAngularVelocityRadPerSec();
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = Math.abs(feederSim.getCurrentDrawAmps());
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelAppliedVolts = volts;
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederAppliedVolts = volts;
  }
}
