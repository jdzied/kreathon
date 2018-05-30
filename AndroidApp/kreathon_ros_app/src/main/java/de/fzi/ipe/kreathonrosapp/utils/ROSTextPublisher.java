package de.fzi.ipe.kreathonrosapp.utils;

import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import de.fzi.ipe.kreathonrosapp.activities.MainActivity;
import std_msgs.String;

public class ROSTextPublisher extends ROSNodeMain {

    private ConnectedNode connectedNode;
    private Publisher<std_msgs.String> textPublisher;

    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_text_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera");
        this.textPublisher = connectedNode.newPublisher(resolver.resolve("android/barcode"), "std_msgs/String");

        for(MainActivity.OnRosNodeRunningListener listener : startListeners) {
            listener.OnRosNodeStarted(this);
        }
    }

    @Override
    public void onShutdown(Node node) {
        textPublisher.shutdown();
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    public void publishText(java.lang.String text) {
        std_msgs.String stringMessage = this.textPublisher.newMessage();
        stringMessage.setData(text);
        textPublisher.publish(stringMessage);
    }
}
