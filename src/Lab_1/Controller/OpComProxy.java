package src.Lab_1.Controller;

import src.Lab_1.*;
import se.lth.control.DoublePoint;

public class OpComProxy implements OpCom_interface {
    private CommServer commServer;

    public OpComProxy(CommServer commServer) {
        this.commServer = commServer;
    }

    public void putControlDataPoint(DoublePoint dp) {
        commServer.putControlDataPoint(dp);
    }

    public void putMeasurementDataPoint(PlotData pd) {
        commServer.putMeasurementDataPoint(pd);
    }
}