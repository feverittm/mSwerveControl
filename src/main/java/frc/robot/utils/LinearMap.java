package frc.robot.utils;

public class LinearMap {
    /**
     * Linear data translation. Copied (with appropriate license) from the Arduino
     * source.
     * 
     * @param Input   Variable
     * @param in_min  expected minimum input value
     * @param in_max  expected maximum input value
     * @param out_min desired minimum output value
     * @param out_max desired maximum output value
     * @return
     */
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
