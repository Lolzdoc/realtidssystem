package src.Lab_1;


import SimEnvironment.AnalogSink;
import SimEnvironment.AnalogSource;

public class BallAndBeamRegul extends Thread {
    private ReferenceGenerator referenceGenerator;
    private PI PIcontroller;
    private PID PIDcontroller;




    // Declarations
    private AnalogSource analogInAngle;
    private AnalogSource analogInPosition;
    private AnalogSink analogOut;
    private AnalogSink analogRef;



    //Define min and max control output
    private double uMin = -10.0;
    private double uMax = 10.0;

    //Constructor
    public BallAndBeamRegul(ReferenceGenerator ref, BallAndBeam bb, int pri) {
        referenceGenerator = ref;
        PIcontroller = new PI("Position PI");
        PIDcontroller = new PID("Angle PID");

        analogInPosition = bb.getSource(0);
        analogInAngle = bb.getSource(1);
        analogOut = bb.getSink(0);
        analogRef = bb.getSink(1);

        setPriority(pri);
    }
    //Saturate output at limits
    private double limit(double u, double umin, double umax) {
        if (u < umin) {
            u = umin;
        } else if (u > umax) {
            u = umax;
        }
        return u;
    }

    public void run() {
        long t = System.currentTimeMillis();
        while (true) {
            // Read inputs
            synchronized (PIDcontroller) {
                double y = analogInPosition.get(); // Get the current ball position from the sensor
                double ref = referenceGenerator.getRef(); // Get the ref Value
                double u = limit(PIDcontroller.calculateOutput(y, ref), uMin, uMax);

                // Update state
                PIDcontroller.updateState(u);


                synchronized (PIcontroller) { // To avoid parameter changes in between
                    // Compute control signal
                    double angle = analogInAngle.get();
                    double v = limit(PIcontroller.calculateOutput(angle, u), uMin, uMax);

                    // Set output
                    analogOut.set(v);

                    // Update state
                    PIcontroller.updateState(v);
                }
                analogRef.set(ref); // Only for the plotter animation

            }

            t = t + PIcontroller.getHMillis();
            long duration = t - System.currentTimeMillis();
            if (duration > 0) {
                try {
                    sleep(duration);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}