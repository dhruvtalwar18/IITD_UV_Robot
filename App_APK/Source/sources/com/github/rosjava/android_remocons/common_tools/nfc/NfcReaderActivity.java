package com.github.rosjava.android_remocons.common_tools.nfc;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
import com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils.ByteArrays;
import com.github.rosjava.android_remocons.common_tools.R;
import java.util.HashMap;

public class NfcReaderActivity extends Activity {
    public static boolean enabled = true;
    private HashMap<String, Object> data;
    private NfcManager nfcManager;
    private TextView textView;

    public void onCreate(Bundle savedState) {
        super.onCreate(savedState);
        try {
            setContentView(R.layout.nfc_tag_scan);
            this.textView = (TextView) findViewById(R.id.text);
            this.textView.setText("Scan a NFC tag");
            this.nfcManager = new NfcManager(this);
        } catch (Exception e) {
            Log.e("NfcReader", e.getMessage());
            finish();
        }
    }

    public void onResume() {
        super.onResume();
        if (this.nfcManager != null) {
            this.nfcManager.enableForegroundDispatch();
        }
    }

    /* access modifiers changed from: protected */
    public void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        if (this.nfcManager != null && this.nfcManager.onNewIntent(intent)) {
            Log.i("NfcReader", "NFC tag read");
            byte[] payload = this.nfcManager.getPayload();
            if (payload.length != 59) {
                Log.e("NfcReader", "Payload doesn't match expected length: " + payload.length + " != " + 56);
                return;
            }
            this.data = new HashMap<>();
            this.data.put("WIFI", ByteArrays.toString(payload, 3, 16).trim());
            int offset = 3 + 16;
            this.data.put("WIFIPW", ByteArrays.toString(payload, offset, 16).trim());
            this.data.put("WIFIENC", "WPA2");
            int offset2 = offset + 16;
            String host = ByteArrays.toString(payload, offset2, 16).trim();
            short port = ByteArrays.toShort(payload, offset2 + 16);
            HashMap<String, Object> hashMap = this.data;
            hashMap.put("URL", "http://" + host + ":" + port);
            finish();
        }
    }

    public void onPause() {
        super.onPause();
        if (this.nfcManager != null) {
            this.nfcManager.disableForegroundDispatch();
        }
    }

    public void finish() {
        Intent returnIntent = new Intent();
        if (this.data != null) {
            returnIntent.putExtra("tag_data", this.data);
            setResult(-1, returnIntent);
        } else {
            setResult(0, returnIntent);
        }
        super.finish();
    }
}
