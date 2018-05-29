package de.fzi.ipe.kreathonrosapp.activities;

import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import de.fzi.ipe.kreathonrosapp.R;
import android.os.Bundle;
import android.util.Log;

import com.mongodb.MongoClient;
import com.mongodb.client.MongoCollection;
import com.mongodb.client.MongoDatabase;
import org.bson.Document;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class MongoDBActivity extends AppCompatActivity {

    String mongoURL_;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_mongo_db);

        //write something to MongoDB
        new AsyncDBwrite().execute();

        try {
            TimeUnit.SECONDS.sleep(1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        //read it again
        new AsyncDBread().execute();
    }
}

class AsyncDBwrite extends AsyncTask<Void, Integer, String>
{
    String TAG = getClass().getSimpleName();

    protected void onPreExecute (){
        super.onPreExecute();
        Log.d(TAG + " PreExceute","On pre Exceute......");
    }

    protected String doInBackground(Void...arg0) {
        Log.d(TAG + " DoINBackGround","On doInBackground...");

        int step = 0;
        int no_step = 5;

        // connect to server
        MongoClient mongoClient = new MongoClient( "192.168.2.117" );
        publishProgress(step++, no_step);

        // read kreathon database on server
        MongoDatabase database = mongoClient.getDatabase("kreathon");
        publishProgress(step++, no_step);

        // get the needed collection from kreathon database on server
        MongoCollection<Document> collection = database.getCollection("android_test");
        publishProgress(step++, no_step);

        Document document = new Document("name", "Café Con Leche")
                .append("contact", new Document("phone", "228-555-0149")
                        .append("email", "cafeconleche@example.com")
                        .append("location",Arrays.asList(-73.92502, 40.8279556)))
                .append("stars", 3)
                .append("categories", Arrays.asList("Bakery", "Coffee", "Pastries"));
        publishProgress(step++, no_step);

        collection.insertOne(document);
        publishProgress(step++, no_step);

        return "You are at PostExecute";
    }

    protected void onProgressUpdate(Integer...a){
        super.onProgressUpdate(a);
        Log.d(TAG + " onProgressUpdate", "You are in progress update ... " + a[0] + " / " + a[1]);
    }

    protected void onPostExecute(String result) {
        super.onPostExecute(result);
        Log.d(TAG + " onPostExecute", "" + result);
    }
}

// read
class AsyncDBread extends AsyncTask<Void, Integer, String>
{
    String TAG = getClass().getSimpleName();

    protected void onPreExecute (){
        super.onPreExecute();
        Log.d(TAG + " PreExceute","On pre Exceute......");
    }

    protected String doInBackground(Void...arg0) {
        Log.d(TAG + " DoINBackGround","On doInBackground...");

        int step = 0;
        int no_step = 5;

        // connect to server
        MongoClient mongoClient = new MongoClient( "192.168.2.117" );
        publishProgress(step++, no_step);

        // read kreathon database on server
        MongoDatabase database = mongoClient.getDatabase("kreathon");
        publishProgress(step++, no_step);

        MongoCollection<Document> collection = database.getCollection("android_test");
        publishProgress(step++, no_step);

        // read first document from collection and print it
        Document myDoc = collection.find().first();
        System.out.println(myDoc.toJson());
        publishProgress(step++, no_step);

        // reading multiple entries or filter the entries look for
        // http://mongodb.github.io/mongo-java-driver/3.4/driver/getting-started/quick-start/

        return "You are at PostExecute";
    }

    protected void onProgressUpdate(Integer...a){
        super.onProgressUpdate(a);
        Log.d(TAG + " onProgressUpdate", "You are in progress update ... " + a[0] + " / " + a[1]);
    }

    protected void onPostExecute(String result) {
        super.onPostExecute(result);
        Log.d(TAG + " onPostExecute", "" + result);
    }
}


