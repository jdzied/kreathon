package de.fzi.ipe.kreathonrosapp.activities;

import android.graphics.Bitmap;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.ImageView;

import org.ros.address.InetAddressFactory;
import org.ros.node.NodeConfiguration;

import java.net.URI;

import de.fzi.ipe.kreathonrosapp.R;
import de.fzi.ipe.kreathonrosapp.utils.MessageAlertFragment;
import de.fzi.ipe.kreathonrosapp.utils.ROSImageSubscriber;
import de.fzi.ipe.kreathonrosapp.utils.ROSNodeMain;

public class ViewImageActivity extends AppCompatActivity {

    private ROSImageSubscriber imageSubscriber;

    private ImageView imageView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_view_image);

        URI rosMasterUri = (URI) getIntent().getExtras().get("MasterURI");

        imageSubscriber = new ROSImageSubscriber();
        imageSubscriber.addOnStartListener(new MainActivity.OnRosNodeRunningListener() {
            @Override
            public void OnRosNodeStarted(ROSNodeMain node) {
                MessageAlertFragment message = MessageAlertFragment.newInstance(ViewImageActivity.this,
                        "Ros node started",
                        "Started ros node " + node.toString());
                message.setPositiveButton("Ok", null);
                message.show(ViewImageActivity.this.getFragmentManager(), "ros_node_start");
            }
        });

        imageSubscriber.setOnNewBitmapListener(new OnNewBitmapListener());
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());

        String nodeName = imageSubscriber.getDefaultNodeName() + "_" + InetAddressFactory.newNonLoopback().getHostAddress().replace(".","");

        nodeConfiguration.setNodeName(nodeName);
        nodeConfiguration.setMasterUri(rosMasterUri);
        MainActivity.nodeMainExecutor.execute(imageSubscriber, nodeConfiguration);

        this.imageView = (ImageView) findViewById(R.id.imageView2);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        MainActivity.nodeMainExecutor.shutdownNodeMain(imageSubscriber);
    }

    public boolean isConnetected() {
        return imageSubscriber.getConnectedNode() != null;
    }

    public class OnNewBitmapListener {
        public void onNewBitmap(final Bitmap b) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    ViewImageActivity.this.imageView.setImageResource(android.R.color.transparent);
                    ViewImageActivity.this.imageView.setImageBitmap(b);
                }
            });
        }
    }
}
