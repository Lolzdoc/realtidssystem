package src.Lab_1;

import se.lth.control.DoublePoint;
import src.Lab_1.Controller.CommServer;

/**
 * Created by hansr on 2016-02-22.
 */
public interface OpCom_interface {

    public void putControlDataPoint(DoublePoint dp);

    public void putMeasurementDataPoint(PlotData pd);


}
