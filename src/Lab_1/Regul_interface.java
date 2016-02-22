package src.Lab_1;


public interface Regul_interface {
    public PIParameters getInnerParameters();

    public void setInnerParameters(PIParameters p);

    public PIDParameters getOuterParameters();

    public void setOuterParameters(PIDParameters p);

    public int getMode();

    public void setBALLMode();

    public void setBEAMMode();

    public void setOFFMode();

    public void shutDown();
}
