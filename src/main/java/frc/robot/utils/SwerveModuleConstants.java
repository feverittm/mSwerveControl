package frc.robot.utils;

public class SwerveModuleConstants {
    public final int moduleNumber;
    public final int driveMotorID;
    public final int angleMotorID;
    public final boolean driveMotorReversed;
    public final boolean angleMotorReversed;
    public final boolean angleEncoderReversed;
    public final double angleEncoderConversionFactor;
    public final double angleEncoderOffset;

    public SwerveModuleConstants(int moduleNumber, int driveMotorID, int angleMotorID, boolean driveMotorReversed,
            boolean angleMotorReversed, boolean angleEncoderReversed, double angleEncoderConversionFactor,
            double angleEncoderOffset) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.driveMotorReversed = driveMotorReversed;
        this.angleMotorReversed = angleMotorReversed;
        this.angleEncoderReversed = angleEncoderReversed;
        this.angleEncoderOffset = angleEncoderOffset;
        this.angleEncoderConversionFactor = angleEncoderConversionFactor;
    }
}

