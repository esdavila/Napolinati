package frc.robot.ClawPivot;


import org.MotorControl.TalonFXMotor;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClawPivotHw extends ClawPivot {
  private TalonFXMotor pivotMotor;
  private DutyCycleEncoder pivotAngle;
  private double encoderOffset;

  public ClawPivotHw() {
    super();
    pivotMotor = new TalonFXMotor("Pivot", 58);
    pivotAngle = new DutyCycleEncoder(1);



    pivotMotor.setSoftLimits(4, 180);
    pivotMotor.setScaleFactor(1 / 7.649);
    pivotMotor.setCurrentLimit(10);
    pivotMotor.setBrakeMode(true);

    pivotMotor.setEncoderPosition(getSensorAngle());
    encoderOffset = 0;  //calibrate if necessary
  }

  @Override
  void setPower(double power) {
    pivotMotor.setPower(power);
  }

  @Override
  void setAngle(double angle) {
    if (pidEnabled) {
      pivotMotor.setPosition(angle);
    } else {
      pivotMotor.setVoltage(0);
    }
  }

  @Override
  public double getSensorAngle() {
      // DutyCycleEncoder.getAbsolutePosition() returns a value from 0.0 to 1.0 for a full revolution.
    // Multiply by 360 to convert to degrees.
    double rawDegrees = pivotAngle.get() * 360.0;
    // Apply the offset (converted to degrees) and wrap the angle between -60° and 300°.
    double adjustedDegrees = rawDegrees - (encoderOffset * 360.0);
    return MathUtil.inputModulus(adjustedDegrees, -60, 300);
  }

  @Override
  public double getAngle() {
    return pivotMotor.getPosition();
  }

  @Override
  void setEncoderPosition(double position) {
    pivotMotor.setEncoderPosition(position);
  }

  @Override
  TalonFX getMotor() {
    return (TalonFX) pivotMotor.getBaseMotor();
  }
}
