package de.fzi.ipe.kreathonrosapp.utils;

import org.ros.node.Node;
import org.ros.node.NodeMain;

import java.util.ArrayList;
import java.util.List;

import de.fzi.ipe.kreathonrosapp.activities.MainActivity;

public abstract class ROSNodeMain implements NodeMain {
    protected List<MainActivity.OnRosNodeRunningListener> startListeners =
            new ArrayList<MainActivity.OnRosNodeRunningListener>();

    protected List<MainActivity.OnRosNodeErrorListener> errorListeners =
            new ArrayList<MainActivity.OnRosNodeErrorListener>();

    public void addOnStartListener(MainActivity.OnRosNodeRunningListener listener) {
        this.startListeners.add(listener);
    }

    public void addOnErrorListener(MainActivity.OnRosNodeErrorListener listener) {
        this.errorListeners.add(listener);
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        for(MainActivity.OnRosNodeErrorListener listener:errorListeners) {
            listener.OnRosNodeError(node, throwable);
        }
    }
}
