public class PIDF {
    private double kP, kI, kD, kF;
    private double integral, lastError;
    private double lastTimestamp;

    public PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF;
        this.lastTimestamp = System.nanoTime() / 1e9;
    }

    public double update(double target, double actual) {
        double now = System.nanoTime() / 1e9;
        double dt = Math.max(1e-6, now - lastTimestamp);
        double error = target - actual;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        lastTimestamp = now;
        return kP*error + kI*integral + kD*derivative + kF*target;
    }

    public void reset() { integral = 0; lastError = 0; }
}
