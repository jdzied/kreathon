package de.fzi.ipe.kreathonrosapp.utils;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.jboss.netty.buffer.ChannelBuffer;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

import de.fzi.ipe.kreathonrosapp.activities.MainActivity;
import de.fzi.ipe.kreathonrosapp.activities.ViewImageActivity;
import sensor_msgs.CompressedImage;

public class ROSImageSubscriber extends ROSNodeMain {

    private ConnectedNode connectedNode;
    private Subscriber<CompressedImage> imageSubscriber;

    private ViewImageActivity.OnNewBitmapListener onBitmapListener;

    private Bitmap lastImage;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_image_receiver");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera");
        this.imageSubscriber = connectedNode.newSubscriber(resolver.resolve("image/compressed"), "sensor_msgs/CompressedImage");
        this.imageSubscriber.addMessageListener(new ROSImageSubscriber.ImageListener());

        for(MainActivity.OnRosNodeRunningListener listener : startListeners) {
            listener.OnRosNodeStarted(this);
        }
    }

    @Override
    public void onShutdown(Node node) {
        imageSubscriber.removeAllMessageListeners();
        imageSubscriber.shutdown();
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    public ConnectedNode getConnectedNode() {
        return connectedNode;
    }

    public Bitmap getLastImage(){
        return lastImage;
    }

    public void setOnNewBitmapListener(ViewImageActivity.OnNewBitmapListener listener) {
        this.onBitmapListener = listener;
    }

    private final class ImageListener implements MessageListener<CompressedImage> {

        @Override
        public void onNewMessage(CompressedImage compressedImage) {
            if(!compressedImage.getFormat().equals("jpeg"))
            {
                return;
            }
            ChannelBuffer jpegBufferData = compressedImage.getData();
            byte[] imageBytes = jpegBufferData.toByteBuffer().array();
            Bitmap image = BitmapFactory.decodeByteArray(imageBytes, jpegBufferData.arrayOffset(), jpegBufferData.readableBytes());
            ROSImageSubscriber.this.lastImage = image;
            ROSImageSubscriber.this.onBitmapListener.onNewBitmap(image);
        }
    }

}
