package de.fzi.ipe.kreathonrosapp.activities;

import android.content.Intent;
import android.net.Uri;
import android.os.Environment;
import android.provider.MediaStore;
import android.support.v4.content.FileProvider;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.text.SimpleDateFormat;
import java.util.Date;

import de.fzi.ipe.kreathonrosapp.R;
import de.fzi.ipe.kreathonrosapp.utils.ROSImagePublisher;

public class MainActivity extends AppCompatActivity {

    private Button takeImageButton;
    private Button viewImageButton;
    private Button scanBarcodeButton;
    private Button mongoButton;

    public final static NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();

    private ROSImagePublisher imagePublisher;

    private String currentPhotoPath;

    static final int REQUEST_IMAGE_CAPTURE = 1;
    static final String imageId = "Ros_Testapp_Image";

    private URI getRosMasterUri() {
        try {
            return new URI(getResources().getString(R.string.ros_master_uri));
        } catch (URISyntaxException e) {
            e.printStackTrace();
            return (URI)null;
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        imagePublisher = new ROSImagePublisher();
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());

        String nodeName = imagePublisher.getDefaultNodeName() + "_" + InetAddressFactory.newNonLoopback().getHostAddress().replace(".","");

        nodeConfiguration.setNodeName(nodeName);
        nodeConfiguration.setMasterUri(getRosMasterUri());
        nodeMainExecutor.execute(imagePublisher, nodeConfiguration);

        takeImageButton = (Button) findViewById(R.id.takeImageButton);
        takeImageButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                takeImage();
            }
        });

        viewImageButton = (Button) findViewById(R.id.viewLastImageButton);
        viewImageButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onViewImageClick();
            }
        });

        mongoButton = (Button) findViewById(R.id.mongoButton);
        mongoButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mongoButtonClick();
            }
        });

        scanBarcodeButton = (Button) findViewById(R.id.scanBarcodeButton);
        scanBarcodeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onScanBarcodeClick();
            }
        });
    }

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == REQUEST_IMAGE_CAPTURE && resultCode == RESULT_OK) {
            File imageFile = new File(currentPhotoPath);
            imagePublisher.sendImage(imageFile);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        nodeMainExecutor.shutdown();
    }

    private File createImageFile(String imageName) throws IOException {
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());

        String imageFileName = imageName;// + "_" + timeStamp;
        File storageDir = getExternalFilesDir(Environment.DIRECTORY_PICTURES);
        File image = File.createTempFile(
                imageFileName,
                ".jpg",
                storageDir
        );

        currentPhotoPath = image.getAbsolutePath();
        return image;
    }

    private void takeImage(){
        Intent takePictureIntent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
        if (takePictureIntent.resolveActivity(getPackageManager()) != null) {
            File imageFile = null;
            try {
                imageFile = createImageFile(imageId);
            } catch (IOException e) {
                e.printStackTrace();
            }

            if (imageFile != null) {
                Uri imageURI = FileProvider.getUriForFile(this,"de.fzi.ipe.kreathonrosapp.fileprovider", imageFile);
                takePictureIntent.putExtra(MediaStore.EXTRA_OUTPUT, imageURI);
                startActivityForResult(takePictureIntent, REQUEST_IMAGE_CAPTURE);
            }
        }
    }

    private void onViewImageClick() {
        Intent intent = new Intent(this, ViewImageActivity.class);
        intent.putExtra("MasterURI", getRosMasterUri());
        startActivity(intent);
    }

    private void onScanBarcodeClick() {
        Intent intent = new Intent(this, BarcodeScannerActivity.class);
        startActivity(intent);
    }

    private void mongoButtonClick() {
        Intent intent = new Intent(this, MongoDBActivity.class);
        intent.putExtra("MongoURL", R.string.mongo_url);
        startActivity(intent);
    }
}
