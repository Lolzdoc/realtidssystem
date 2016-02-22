package src.BeamAndBall;


import SimEnvironment.AnalogSink;
import SimEnvironment.AnalogSource;

public class BallAndBeamRegul extends Thread {
    private ReferenceGenerator referenceGenerator;
    private PI PIcontroller;
    private PID PIDcontroller;

    private AnalogSource analogIn;
    private AnalogSink analogOut;
    private AnalogSink analogRef;


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
            double y = analogIn.get();
            double ref = referenceGenerator.getRef();

            synchronized (PIcontroller) { // To avoid parameter changes in between
                // Compute control signal
                double u = limit(PIcontroller.calculateOutput(y, ref), uMin, uMax);

                // Set output
                analogOut.set(u);

                // Update state
                PIcontroller.updateState(u);
            }

            synchronized (PIcontroller) { // To avoid parameter changes in between
                // Compute control signal
                double u = limit(PIcontroller.calculateOutput(y, ref), uMin, uMax);

                // Set output
                analogOut.set(u);

                // Update state
                PIcontroller.updateState(u);
            }
            analogRef.set(ref); // Only for the plotter animation

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