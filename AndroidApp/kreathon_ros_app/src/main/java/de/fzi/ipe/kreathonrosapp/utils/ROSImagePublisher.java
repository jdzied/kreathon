package de.fzi.ipe.kreathonrosapp.utils;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import de.fzi.ipe.kreathonrosapp.activities.MainActivity;
import sensor_msgs.CompressedImage;

public class ROSImagePublisher extends ROSNodeMain {

    private ConnectedNode connectedNode;
    private Publisher<CompressedImage> imagePublisher;

    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_image_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera");
        this.imagePublisher = connectedNode.newPublisher(resolver.resolve("image/compressed"), "sensor_msgs/CompressedImage");

        for(MainActivity.OnRosNodeRunningListener listener : startListeners) {
            listener.OnRosNodeStarted(this);
        }
    }

    @Override
    public void onShutdown(Node node) {
        imagePublisher.shutdown();
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    public ConnectedNode getConnectedNode() {
        return this.connectedNode;
    }

    private static byte[] getRawBytesFromFile(String path) throws FileNotFoundException, IOException {

        byte[] image;
        File file = new File(path);
        image = new byte[(int)file.length()];

        FileInputStream fileInputStream = new FileInputStream(file);
        fileInputStream.read(image);

        return image;
    }

    public void sendImage(File imageFile) {
        ChannelBufferOutputStream bufferStream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());

        byte[] imageBytes = new byte[0];

        try {
            imageBytes = getRawBytesFromFile(imageFile.getAbsolutePath());
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            bufferStream.write(imageBytes);
        } catch (IOException e) {
            e.printStackTrace();
        }

        Time currentTime = this.connectedNode.getCurrentTime();
        String frameId = "camera";
        CompressedImage image = this.imagePublisher.newMessage();
        image.setFormat("jpeg");
        image.getHeader().setStamp(currentTime);
        image.getHeader().setFrameId(frameId);
        image.setData(bufferStream.buffer().copy());

        imagePublisher.publish(image);
    }
}
