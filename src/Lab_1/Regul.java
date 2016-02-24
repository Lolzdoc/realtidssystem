package src.Lab_1;

import se.lth.control.*;
import se.lth.control.realtime.*;

/** Dummy Regul class for exercise 4. Generates and sends sinewaves to OpCom
 and replies with print-outs when the set methods are called. */
public class Regul extends Thread implements Regul_interface {
    public static final int OFF=0, BEAM=1, BALL=2;

    private PIParameters innerPar;
    private PIDParameters outerPar;
    private OpCom_interface opcom;

    private int mode;
    private static final long h = 100; // period (ms)
    private static final double twoPI = 2 * Math.PI;
    //Define min and max control output
    private double uMin = -10.0;
    private double uMax = 10.0;

    private double realTime = 0.0;

    private AnalogIn analogInAngle;
    private AnalogIn analogInPosition;
    private AnalogOut analogOut;

    private RefGen_interface r;

    private PI inner_PI;
    private PID outer_PID;

    private boolean doIt = true;

    /** Constructor. Sets initial values of the controller parameters and initial mode. */
    public Regul(int priority) {
        inner_PI = new PI("inner PI Parameters");
        outer_PID = new PID("outer PID Parameters");

        try {
            analogInPosition = new AnalogIn(1);
            analogInAngle = new AnalogIn(0);
            analogOut = new AnalogOut(0);
        } catch (IOChannelException e) {
            e.printStackTrace();
        }


        innerPar = new PIParameters();
        innerPar.K = 1.8;
        innerPar.Ti = 1;
        innerPar.Tr = 10.0;
        innerPar.Beta = 1.0;
        innerPar.H = 0.045;
        innerPar.integratorOn = true;

        outerPar = new PIDParameters();


        outerPar.K = -(0.2);
        outerPar.Ti = 1;
        outerPar.Td = 1.3;
        outerPar.Tr = 10.0;
        outerPar.N = 10.0;
        outerPar.Beta = 1.0;
        outerPar.H = 0.05;
        outerPar.integratorOn = true;

        inner_PI.setParameters(innerPar);
        outer_PID.setParameters(outerPar);
        mode = OFF;


        setPriority(priority);
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

    private void ball_controller(){
        try {
        synchronized (outer_PID) {
            double  y = analogInPosition.get(); // Get the current ball position from the sensor
            double ref = r.getRef(); // Get the ref Value
            double u = limit(outer_PID.calculateOutput(y, ref), uMin, uMax);

            // Update state
            outer_PID.updateState(u);


            synchronized (inner_PI) { // To avoid parameter changes in between
                // Compute control signal
                double angle = analogInAngle.get();
                double v = limit(inner_PI.calculateOutput(angle, u), uMin, uMax);

                analogOut.set(v);
                // Update state
                inner_PI.updateState(v);
            }
            opcom.putControlDataPoint(new DoublePoint(realTime,u));
            opcom.putMeasurementDataPoint(new PlotData(realTime,ref,y)); // Only for the plotter animation

            realTime += ((double) h)/1000.0;


        }
        } catch (IOChannelException e) {
            e.printStackTrace();
        }
    }
    private void beam_controller(){
        double y = 0;
        double u;
        try {
            y = analogInAngle.get();
        } catch (IOChannelException e) {
            e.printStackTrace();
        }
        double ref = r.getRef();

        synchronized (inner_PI) { // To avoid parameter changes in between
            // Compute control signal
            u = limit(inner_PI.calculateOutput(y, ref), uMin, uMax);

            // Set output
            try {
                analogOut.set(u);
            } catch (IOChannelException e) {
                e.printStackTrace();
            }

            // Update state
            inner_PI.updateState(u);
        }
        opcom.putControlDataPoint(new DoublePoint(realTime,u));
        opcom.putMeasurementDataPoint(new PlotData(realTime,ref,y)); // Only for the plotter animation

        realTime += ((double) h)/1000.0;
    }
    private void Regul_sleep(long time){
        if (time > 0) {
            try {
                sleep(time);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    /** Run method. Sends data periodically to OpCom. */
    public void run() {

        long duration;
        long t;


        while (doIt) {
            t  = System.currentTimeMillis();
            switch (mode) {
                case BEAM:
                    beam_controller();
                    t = t + inner_PI.getHMillis();
                    duration = t - System.currentTimeMillis();
                    Regul_sleep(duration);
                    break;

                case BALL:
                    ball_controller();
                    t = t + inner_PI.getHMillis() + outer_PID.getHMillis();
                    duration = t - System.currentTimeMillis();
                    Regul_sleep(duration);
                    break;

                default:
                    opcom.putControlDataPoint(new DoublePoint(realTime,0));
                    opcom.putMeasurementDataPoint(new PlotData(realTime,0,0)); // Only for the plotter animation
                    realTime += ((double) h)/1000.0;

                    try {
                        analogOut.set(OFF);
                    } catch (IOChannelException e) {
                        e.printStackTrace();
                        return;
                    }

                    t = t + inner_PI.getHMillis() + outer_PID.getHMillis();
                    duration = t - System.currentTimeMillis();
                    Regul_sleep(duration);

                    break;

            }


        }
    }
    /** Stops the thread. */
    private void stopThread() {

        doIt = false;
    }

    /** Called by OpCom to set the parameter values of the inner loop. */
    public synchronized void setInnerParameters(PIParameters p) {
        inner_PI.setParameters(p);
        System.out.println("Parameters changed for inner loop");
    }

    /** Called by OpCom during initialization to get the parameter values of the inner loop. */
    public synchronized PIParameters getInnerParameters() {
        return inner_PI.getParameters();
    }

    /** Called by OpCom to set the parameter values of the outer loop */
    public synchronized void setOuterParameters(PIDParameters p) {
        outer_PID.setParameters(p);
        System.out.println("Parameters changed for outer loop");
    }

    /** Called by OpCom during initialization to get the parameter values of the outer loop. */
    public synchronized PIDParameters getOuterParameters() {
        return outer_PID.getParameters();
    }

    /** Called by OpCom to turn off the controller. */
    public synchronized void setOFFMode() {
        mode = OFF;
        inner_PI.reset();
        outer_PID.reset();
        System.out.println("Controller turned OFF");
    }

    /** Called by OpCom to set the Controller in BEAM mode. */
    public synchronized void setBEAMMode() {
        mode = BEAM;
        inner_PI.reset();
        outer_PID.reset();
        System.out.println("Controller in BEAM mode"
);
    }

    /** Called by OpCom to set the Controller in BALL mode. */
    public synchronized void setBALLMode() {
        mode = BALL;
        inner_PI.reset();
        outer_PID.reset();
        System.out.println("Controller in BALL mode");
    }

    /** Called by OpCom during initialization to get the initail mode of the controller. */
    public synchronized int getMode() {
        return mode;
    }

    /** Called by OpCom when the Stop button is pressed. */
    public synchronized void shutDown() {
        stopThread();
    }

    /** Sets up a reference to OpCom. Called from Main. */
    public void setOpCom(OpCom_interface o) {
        opcom = o;
    }

    // Passes in a reference to ReferenceGenerator. Called from Main.
    public void setRefGen(RefGen_interface r){
        this.r = r;
    }

}

