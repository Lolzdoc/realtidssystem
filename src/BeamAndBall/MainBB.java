package src.BeamAndBall;

public class MainBB {
    public static void main(String[] argv) {
        final int PIDregulPriority = 8;

        BallAndBeam bb = new BallAndBeam();
        ReferenceGenerator refgen = new ReferenceGenerator(20.0, 4.0);
        BallAndBeamRegul PIDregul = new BallAndBeamRegul(refgen, bb, PIDregulPriority);

        refgen.start();
        PIDregul.start();
    }
}