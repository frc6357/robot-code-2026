package frc.lib.utils.filters;

/**
 * A deadband filter to be applied for joysticks/inputs that applies a deadband
 * and maintains a linear shape when calculating the processed input
 */
public class LinearDeadbandFilter implements Filter {
    private double deadband;
    private double maxInput;
    private double slope;

    /**
     * Creates a new linear deadband filter
     * @param deadband The deadband to apply
     * @param maxInput The maximum expected input of the joystick/input
     */
    public LinearDeadbandFilter(double deadband, double maxInput) {
        this.deadband = deadband;
        this.maxInput = maxInput;
        slope = 1 / (maxInput - deadband);
    }

    @Override
    public double filter(double rawAxis) {
        // If the value ends up negative, the magnitude of the joystick input can be ingnored
        double absoluteAxis = slope * (Math.abs(rawAxis) - deadband);
        return (absoluteAxis > 0.0) ? (Math.signum(rawAxis) * absoluteAxis) : 0.0;
    }
    
    public void setDeadband(double deadband) {
        this.deadband = deadband;
        slope = 1 / (maxInput - deadband);
    }
}
