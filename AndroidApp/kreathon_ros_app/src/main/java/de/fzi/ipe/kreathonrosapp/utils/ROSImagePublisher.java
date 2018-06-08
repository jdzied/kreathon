package de.fzi.ipe.kreathonrosapp.utils;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;

import org.apache.commons.imaging.common.IImageMetadata;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import org.apache.commons.imaging.ImageReadException;
import org.apache.commons.imaging.Imaging;
import org.apache.commons.imaging.formats.jpeg.JpegImageMetadata;
import org.apache.commons.imaging.formats.tiff.TiffField;
import org.apache.commons.imaging.formats.tiff.constants.TiffTagConstants;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import de.fzi.ipe.kreathonrosapp.activities.MainActivity;
import sensor_msgs.CompressedImage;
import sensor_msgs.CameraInfo;

public class ROSImagePublisher extends ROSNodeMain {

    private ConnectedNode connectedNode;
    private Publisher<CompressedImage> imagePublisher;
    private Publisher<CameraInfo> cameraInfoPublisher;

    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_image_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera");
        this.imagePublisher = connectedNode.newPublisher(resolver.resolve("image/compressed"), "sensor_msgs/CompressedImage");
        this.cameraInfoPublisher = connectedNode.newPublisher(resolver.resolve("camera_info"), "sensor_msgs/CameraInfo");
        for(MainActivity.OnRosNodeRunningListener listener : startListeners) {
            listener.OnRosNodeStarted(ROSImagePublisher.this);
        }
    }

    @Override
    public void onShutdown(Node node) {
        imagePublisher.shutdown();
    }

    @Override
    public void onShutdownComplete(Node node) {
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

        try {
            final IImageMetadata metadata = Imaging.getMetadata(imageFile);
            final JpegImageMetadata jpegMetadata = (JpegImageMetadata) metadata;

            int orientationIntValue = 1;

            if(jpegMetadata != null) {
                TiffField tiff_orientation = jpegMetadata.findEXIFValue(TiffTagConstants.TIFF_TAG_ORIENTATION);
                if(tiff_orientation != null)
                    orientationIntValue = tiff_orientation.getIntValue();
            }

            if (orientationIntValue == 6) {
                Bitmap image = BitmapFactory.decodeFile(imageFile.getAbsolutePath());
                Matrix matrix = new Matrix();
                matrix.setRotate(90);
                Bitmap rotated = Bitmap.createBitmap(image, 0, 0, image.getWidth(), image.getHeight(), matrix, true);
                rotated.compress(Bitmap.CompressFormat.JPEG, 100, bufferStream);
            }
            else {
                byte[] imageBytes = new byte[0];
                imageBytes = getRawBytesFromFile(imageFile.getAbsolutePath());
                bufferStream.write(imageBytes);
            }
        } catch (ImageReadException e) {
            e.printStackTrace();
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
