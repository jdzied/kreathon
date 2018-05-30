package de.fzi.ipe.kreathonrosapp.utils;

import android.app.AlertDialog;
import android.app.Dialog;
import android.app.DialogFragment;
import android.content.Context;
import android.content.DialogInterface;
import android.os.Bundle;

public class MessageAlertFragment extends DialogFragment {

    AlertDialog.Builder dialogBuilder;

    public static MessageAlertFragment newInstance(Context context,
                                                   String title,
                                                   String message) {
        MessageAlertFragment messageFragment = new MessageAlertFragment();
        messageFragment.dialogBuilder = new AlertDialog.Builder(context);
        messageFragment.setMessage(title, message);
        return messageFragment;
    }

    public void setMessage(String title, String message) {
        dialogBuilder.setMessage(message)
                .setTitle(title);
    }

    public void setPositiveButton(String caption,
                                  DialogInterface.OnClickListener listener) {
        dialogBuilder.setPositiveButton(caption, listener);
    }

    public void setNegativeButton(String caption,
                                  DialogInterface.OnClickListener listener) {
        dialogBuilder.setNegativeButton(caption, listener);
    }

    public void setNeutralButton(String caption,
                                 DialogInterface.OnClickListener listener) {
        dialogBuilder.setNeutralButton(caption, listener);
    }

    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState) {
        return dialogBuilder.create();
    }


}
