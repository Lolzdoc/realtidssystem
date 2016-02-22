package src.BeamAndBall;

public class Main {
    public static void main(String[] argv) {
        final int regulPriority = 8;

        Beam Beam = new Beam();
        ReferenceGenerator refgen = new ReferenceGenerator(20.0, 4.0);
        BeamRegul regul = new BeamRegul(refgen, Beam, regulPriority);

        refgen.start();
        regul.start();
    }
}