package de.fzi.ipe.kreathonrosapp.utils;

import org.ros.node.NodeMain;

import java.util.ArrayList;
import java.util.List;

import de.fzi.ipe.kreathonrosapp.activities.MainActivity;

public abstract class ROSNodeMain implements NodeMain {
    protected List<MainActivity.OnRosNodeRunningListener> startListeners =
            new ArrayList<MainActivity.OnRosNodeRunningListener>();

    public void addOnStartListener(MainActivity.OnRosNodeRunningListener listener) {
        this.startListeners.add(listener);
    }
}
