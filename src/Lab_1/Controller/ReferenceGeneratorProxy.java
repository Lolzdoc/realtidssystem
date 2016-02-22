package src.Lab_1.Controller;

import src.Lab_1.RefGen_interface;
public class ReferenceGeneratorProxy implements RefGen_interface {
    private CommServer commServer;

    public ReferenceGeneratorProxy(CommServer commServer) {
        this.commServer = commServer;
    }

    public double getRef() {
        return commServer.getRef();
    }
}