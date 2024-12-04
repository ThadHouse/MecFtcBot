package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ArmSubsystem extends SubsystemBase {
    DutyCycleEncoder m_encoder = new DutyCycleEncoder(9, 360, 259);
    PWMSparkMax m_motor = new PWMSparkMax(0);

    public ArmSubsystem() {
        m_encoder.setAssumedFrequency(968);
        m_encoder.setInverted(true);
    }

    @Override
    public void periodic() {
    }

    public void drive(double power) {
        m_motor.set(power);
    }

    public Angle getArmPosition() {
        return Degree.of(MathUtil.inputModulus(m_encoder.get(), -90, 270));
    }
}
