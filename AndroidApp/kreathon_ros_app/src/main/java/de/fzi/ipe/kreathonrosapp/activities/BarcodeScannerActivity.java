package de.fzi.ipe.kreathonrosapp.activities;

import de.fzi.ipe.kreathonrosapp.utils.MessageAlertFragment;
import de.fzi.ipe.kreathonrosapp.utils.ROSNodeMain;
import de.fzi.ipe.kreathonrosapp.utils.ROSTextPublisher;
import me.dm7.barcodescanner.zxing.ZXingScannerView;

import android.content.DialogInterface;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.support.v4.app.DialogFragment;
import android.os.Bundle;
import android.view.ViewGroup;
import com.google.zxing.BarcodeFormat;
import com.google.zxing.Result;

import org.ros.address.InetAddressFactory;
import org.ros.node.NodeConfiguration;

import java.net.URI;
import java.util.ArrayList;
import java.util.List;

import de.fzi.ipe.kreathonrosapp.R;

public class BarcodeScannerActivity extends AppCompatActivity implements ZXingScannerView.ResultHandler {
    private ZXingScannerView mScannerView;
    private ArrayList<Integer> mSelectedIndices;

    private ROSTextPublisher textPublisher;

    @Override
    public void onCreate(Bundle state) {
        super.onCreate(state);

        URI rosMasterUri = (URI) getIntent().getExtras().get("MasterURI");

        textPublisher = new ROSTextPublisher();
        textPublisher.addOnStartListener(new MainActivity.OnRosNodeRunningListener() {
            @Override
            public void OnRosNodeStarted(ROSNodeMain node) {
                MessageAlertFragment message = MessageAlertFragment.newInstance(BarcodeScannerActivity.this,
                        "Ros node started",
                        "Started ros node " + node.toString());
                message.show(BarcodeScannerActivity.this.getFragmentManager(), "ros_node_start");
            }
        });

        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());

        String nodeName = textPublisher.getDefaultNodeName() + "_" + InetAddressFactory.newNonLoopback().getHostAddress().replace(".","");

        nodeConfiguration.setNodeName(nodeName);
        nodeConfiguration.setMasterUri(rosMasterUri);
        MainActivity.nodeMainExecutor.execute(textPublisher, nodeConfiguration);

        setContentView(R.layout.activity_barcode_sanner);
        ViewGroup contentFrame = (ViewGroup) findViewById(R.id.barcodeScannerContent);
        mScannerView = new ZXingScannerView(this);
        setupFormats();
        contentFrame.addView(mScannerView);
    }

    @Override
    public void onResume() {
        super.onResume();
        mScannerView.setResultHandler(this);
        mScannerView.startCamera();
        mScannerView.resumeCameraPreview(this);
    }

    @Override
    public void onPause() {
        super.onPause();
        mScannerView.stopCamera();

    }

    public void setupFormats() {
        List<BarcodeFormat> formats = new ArrayList<BarcodeFormat>();
        if(mSelectedIndices == null || mSelectedIndices.isEmpty()) {
            mSelectedIndices = new ArrayList<Integer>();
            for(int i = 0; i < ZXingScannerView.ALL_FORMATS.size(); i++) {
                mSelectedIndices.add(i);
            }
        }

        for(int index : mSelectedIndices) {
            formats.add(ZXingScannerView.ALL_FORMATS.get(index));
        }
        if(mScannerView != null) {
            mScannerView.setFormats(formats);
        }
    }

    @Override
    public void handleResult(final Result rawResult) {
        String message = "Contents = " + rawResult.getText() + ", Format = " + rawResult.getBarcodeFormat().toString();
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage(message)
                .setTitle("Barcode Read");

        builder.setPositiveButton("Publish", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                BarcodeScannerActivity.this.publishText(rawResult.getText());
                mScannerView.resumeCameraPreview(BarcodeScannerActivity.this);
            }
        });
        builder.setNegativeButton("Close", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                mScannerView.resumeCameraPreview(BarcodeScannerActivity.this);
            }
        });

        builder.create().show();
    }

    private void publishText(String text){
        textPublisher.publishText(text);
    }
}
