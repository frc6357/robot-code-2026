package frc.lib.utils.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveStickFilter implements Filter {
    private LinearDeadbandFilter deadbandFilter;
    private SlewRateLimiter slewRateFilter;
        
        public DriveStickFilter(double slewRate, double deadband) {                
            setDeadband(deadband, 1.0);
            setSlewRate(slewRate);
        }
        public DriveStickFilter(double slewRate, LinearDeadbandFilter deadbandFilter) {                
            this.deadbandFilter = deadbandFilter;
            setSlewRate(slewRate);
        }
    
        @Override
        public double filter(double rawAxis) {
            double processedAxis = deadbandFilter.filter(rawAxis);
            double slewedAxis = slewRateFilter.calculate(processedAxis);
            return Math.min(slewedAxis, 1.0);
        }

        public void setDeadband(double deadband, double maxInput) {
            this.deadbandFilter = new LinearDeadbandFilter(deadband, maxInput);
        }
    
        public void setSlewRate(double slewRate) {
            this.slewRateFilter = new SlewRateLimiter(slewRate);
        }    
}