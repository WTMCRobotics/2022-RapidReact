import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;

class LimitedMotor {
    // the motor being limited
    private CANSparkMax spark;
    // the encoder that belongs to the above spark
    private RelativeEncoder encoder;
    // controls the spark
    private SparkMaxPIDController pidController;

    // the position of the encoder in rotations when it hits the reverse limit switch.
    private double start;
    // the distance in rotations from the forward to the reverse limit switch.
    private final double range;
    // the maximum speed, in RPM, at which it is safe to run into the limit switch
    private final double safeSpeed;

    // Tells us whether we have found where the reverse limit switch is
    private boolean startIsKnown = false;

    // where we want it to go to
    // Where 1 is the forward limit switch and 0 is the reverse limit switch
    private double targetPos;

    /**
     * Creates a new limited motor
     * 
     * @param spark the motor to be limited
     * @param range the distance in rotations from the forward to the reverse limit switch
     * @param safeSpeed the maximum speed, in RPM, at which it is safe to run into the limit switch
     */
    public LimitedMotor(CANSparkMax spark, double range, double safeSpeed) {
        this.spark = spark;
        this.range = range;
        this.safeSpeed = safeSpeed;
        // 4096 is the ticks per revolution of the encoder
        this.encoder = spark.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
        this.pidController = spark.getPIDController();
    }

    /**
     * this method should be called in robot periodic
     * moves the motor closer to the target position
     */
    public void tick() {
        
        if (!this.startIsKnown){
            this.spark.setReference(this.safeSpeed * -1, CANSparkMax.ControlType.kSmartVelocity);
            this.start = this.encoder.getPosition(); // This is in rotations, but that can be changed.
            this.startIsKnown = true;
        } else {
            this.spark.setReference(this.start + (this.range) * this.targetPos, CANSparkMax.ControlType.kSmartMotion);
        }
    }

    /**
     * @return where we want it to go to
     * Where 1 is the forward limit switch and 0 is the reverse limit switch
     */
    public double getTargetPos() {
        return this.targetPos;
    }

    /**
     * @param targetPos where we want it to go to
     * Where 1 is the forward limit switch and 0 is the reverse limit switch
     */
    public void setTargetPos(double targetPos){
        if (targetPos < 0) {
            this.targetPos = 0;
        } else if (targetPos > 1) {
            this.targetPos = 1;
        } else {
            this.targetPos = targetPos;
        }
    }
}