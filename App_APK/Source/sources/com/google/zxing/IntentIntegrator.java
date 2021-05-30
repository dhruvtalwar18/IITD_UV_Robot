package com.google.zxing;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.DialogInterface;
import android.content.Intent;
import android.net.Uri;

public final class IntentIntegrator {
    public static final String ALL_CODE_TYPES = null;
    public static final String DEFAULT_MESSAGE = "This application requires Barcode Scanner. Would you like to install it?";
    public static final String DEFAULT_NO = "No";
    public static final String DEFAULT_TITLE = "Install Barcode Scanner?";
    public static final String DEFAULT_YES = "Yes";
    public static final String ONE_D_CODE_TYPES = "UPC_A,UPC_E,EAN_8,EAN_13,CODE_39,CODE_93,CODE_128";
    private static final String PACKAGE = "com.google.zxing.client.android";
    public static final String PRODUCT_CODE_TYPES = "UPC_A,UPC_E,EAN_8,EAN_13";
    public static final String QR_CODE_TYPES = "QR_CODE";
    public static final int REQUEST_CODE = 195543262;

    private IntentIntegrator() {
    }

    public static AlertDialog initiateScan(Activity activity) {
        return initiateScan(activity, (CharSequence) DEFAULT_TITLE, (CharSequence) DEFAULT_MESSAGE, (CharSequence) DEFAULT_YES, (CharSequence) DEFAULT_NO);
    }

    public static AlertDialog initiateScan(Activity activity, int stringTitle, int stringMessage, int stringButtonYes, int stringButtonNo) {
        return initiateScan(activity, (CharSequence) activity.getString(stringTitle), (CharSequence) activity.getString(stringMessage), (CharSequence) activity.getString(stringButtonYes), (CharSequence) activity.getString(stringButtonNo));
    }

    public static AlertDialog initiateScan(Activity activity, CharSequence stringTitle, CharSequence stringMessage, CharSequence stringButtonYes, CharSequence stringButtonNo) {
        return initiateScan(activity, stringTitle, stringMessage, stringButtonYes, stringButtonNo, ALL_CODE_TYPES);
    }

    public static AlertDialog initiateScan(Activity activity, CharSequence stringTitle, CharSequence stringMessage, CharSequence stringButtonYes, CharSequence stringButtonNo, CharSequence stringDesiredBarcodeFormats) {
        Intent intentScan = new Intent("com.google.zxing.client.android.SCAN");
        intentScan.setPackage(PACKAGE);
        intentScan.addCategory("android.intent.category.DEFAULT");
        if (stringDesiredBarcodeFormats != null) {
            intentScan.putExtra("SCAN_FORMATS", stringDesiredBarcodeFormats);
        }
        try {
            activity.startActivityForResult(intentScan, REQUEST_CODE);
            return null;
        } catch (ActivityNotFoundException e) {
            return showDownloadDialog(activity, stringTitle, stringMessage, stringButtonYes, stringButtonNo);
        }
    }

    private static AlertDialog showDownloadDialog(final Activity activity, CharSequence stringTitle, CharSequence stringMessage, CharSequence stringButtonYes, CharSequence stringButtonNo) {
        AlertDialog.Builder downloadDialog = new AlertDialog.Builder(activity);
        downloadDialog.setTitle(stringTitle);
        downloadDialog.setMessage(stringMessage);
        downloadDialog.setPositiveButton(stringButtonYes, new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialogInterface, int i) {
                activity.startActivity(new Intent("android.intent.action.VIEW", Uri.parse("market://details?id=com.google.zxing.client.android")));
            }
        });
        downloadDialog.setNegativeButton(stringButtonNo, new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialogInterface, int i) {
            }
        });
        return downloadDialog.show();
    }

    public static IntentResult parseActivityResult(int requestCode, int resultCode, Intent intent) {
        if (requestCode != 195543262) {
            return null;
        }
        if (resultCode == -1) {
            return new IntentResult(intent.getStringExtra("SCAN_RESULT"), intent.getStringExtra("SCAN_RESULT_FORMAT"));
        }
        return new IntentResult((String) null, (String) null);
    }

    public static void shareText(Activity activity, CharSequence text) {
        shareText(activity, text, (CharSequence) DEFAULT_TITLE, (CharSequence) DEFAULT_MESSAGE, (CharSequence) DEFAULT_YES, (CharSequence) DEFAULT_NO);
    }

    public static void shareText(Activity activity, CharSequence text, int stringTitle, int stringMessage, int stringButtonYes, int stringButtonNo) {
        shareText(activity, text, (CharSequence) activity.getString(stringTitle), (CharSequence) activity.getString(stringMessage), (CharSequence) activity.getString(stringButtonYes), (CharSequence) activity.getString(stringButtonNo));
    }

    public static void shareText(Activity activity, CharSequence text, CharSequence stringTitle, CharSequence stringMessage, CharSequence stringButtonYes, CharSequence stringButtonNo) {
        Intent intent = new Intent();
        intent.setAction("com.google.zxing.client.android.ENCODE");
        intent.setPackage(PACKAGE);
        intent.putExtra("ENCODE_TYPE", "TEXT_TYPE");
        intent.putExtra("ENCODE_DATA", text);
        try {
            activity.startActivity(intent);
        } catch (ActivityNotFoundException e) {
            showDownloadDialog(activity, stringTitle, stringMessage, stringButtonYes, stringButtonNo);
        }
    }
}
