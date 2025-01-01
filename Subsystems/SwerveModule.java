package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkLowLevel;




public class SwerveModule{
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final CANEncoder driveEncoder;
  private final CANEncoder turningEncoder;

  private final PIDController turningPidController;

  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,boolean turningMotorReversed,
  int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.absoluteEncoderOffsetRad=absoluteEncoderOffset;
    this.absoluteEncoderReversed=absoluteEncoderReversed;
    absoluteEncoder=new AnalogInput(absoluteEncoderId);

    driveMotor=new CANSparkMax(driveMotorId, CANSparkLowLevel.MotorType.kBrushless);
    turningMotor=new CANSparkMax(turningMotorId, CANSparkLowLevel.MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder=driveMotor.getEncoder();
    turningEncoder=turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPidController=new PIDController(Constants.ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(Math.PI, Math.PI);

    resetEncoders();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  /*public double getAbsoluteEncoderRad() {
    double angle=absoluteEncoder.getVoltage()/RobotController.getVoltage5V();
    angle*=2.0*Math.PI;
    angle-=absoluteEncoderOffsetRad;
    return angle*(absoluteEncoderReversed ? -1.0 : 1.0);
  }*/
  public Rotation2d getAbsoluteEncoderRad() {
    double angle=absoluteEncoder.getVoltage()/RobotController.getVoltage5V();
    angle*=2.0*Math.PI;
    angle-=absoluteEncoderOffsetRad;
    return new Rotation2d(angle*(absoluteEncoderReversed ? -1.0 : 1.0));
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad().getRadians());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond)<.001) {
      stop();
    }
    state=SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond/Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
    SmartDashboard.putString("Swerve[]"+absoluteEncoder.getChannel()+"] state",state.toString());

  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}