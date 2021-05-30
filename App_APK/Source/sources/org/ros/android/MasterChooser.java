package org.ros.android;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.AutoCompleteTextView;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.Toast;
import com.google.common.base.Preconditions;
import com.google.zxing.IntentIntegrator;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;
import org.ros.EnvironmentVariables;
import org.ros.android.android_core_components.R;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.node.client.MasterClient;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;

public class MasterChooser extends AppCompatActivity {
    private static final String BAR_CODE_SCANNER_PACKAGE_NAME = "com.google.zxing.client.android.SCAN";
    private static final String CONNECTION_EXCEPTION_TEXT = "ECONNREFUSED";
    private static final int DEFAULT_PORT = 11311;
    private static final String PREFS_KEY_NAME = "URI_KEY";
    private static final String RECENT_COUNT_KEY_NAME = "RECENT_MASTER_URI_COUNT";
    private static final int RECENT_MASTER_HISTORY_COUNT = 5;
    private static final String RECENT_PREFIX_KEY_NAME = "RECENT_MASTER_URI_";
    private static final String UNKNOW_HOST_TEXT = "UnknownHost";
    /* access modifiers changed from: private */
    public Button connectButton;
    /* access modifiers changed from: private */
    public LinearLayout connectionLayout;
    /* access modifiers changed from: private */
    public String selectedInterface;
    /* access modifiers changed from: private */
    public AutoCompleteTextView uriText;

    private class StableArrayAdapter extends ArrayAdapter<String> {
        HashMap<String, Integer> idMap = new HashMap<>();

        public StableArrayAdapter(Context context, int textViewResourceId, List<String> objects) {
            super(context, textViewResourceId, objects);
            for (int i = 0; i < objects.size(); i++) {
                this.idMap.put(objects.get(i), Integer.valueOf(i));
            }
        }

        public long getItemId(int position) {
            return (long) this.idMap.get((String) getItem(position)).intValue();
        }

        public boolean hasStableIds() {
            return true;
        }
    }

    /* access modifiers changed from: protected */
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.master_chooser);
        final Pattern uriPattern = RosURIPattern.URI;
        this.uriText = (AutoCompleteTextView) findViewById(R.id.master_chooser_uri);
        this.connectButton = (Button) findViewById(R.id.master_chooser_ok);
        this.uriText.setThreshold(RosURIPattern.HTTP_PROTOCOL_LENGTH);
        this.uriText.setAdapter(new ArrayAdapter<>(this, 17367057, getRecentMasterURIs()));
        this.uriText.addTextChangedListener(new TextWatcher() {
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                if (!uriPattern.matcher(s.toString()).matches()) {
                    MasterChooser.this.uriText.setError("Please enter valid URI");
                    MasterChooser.this.connectButton.setEnabled(false);
                    return;
                }
                MasterChooser.this.uriText.setError((CharSequence) null);
                MasterChooser.this.connectButton.setEnabled(true);
            }

            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
            }

            public void afterTextChanged(Editable s) {
            }
        });
        ListView interfacesList = (ListView) findViewById(R.id.networkInterfaces);
        List<String> list = new ArrayList<>();
        try {
            Iterator<T> it = Collections.list(NetworkInterface.getNetworkInterfaces()).iterator();
            while (it.hasNext()) {
                NetworkInterface networkInterface = (NetworkInterface) it.next();
                if (networkInterface.isUp() && !networkInterface.isLoopback()) {
                    list.add(networkInterface.getName());
                }
            }
            this.selectedInterface = "";
            interfacesList.setAdapter(new StableArrayAdapter(this, 17367043, list));
            interfacesList.setOnItemClickListener(new AdapterView.OnItemClickListener() {
                public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                    String unused = MasterChooser.this.selectedInterface = parent.getItemAtPosition(position).toString();
                    MasterChooser masterChooser = MasterChooser.this;
                    masterChooser.toast("Using " + MasterChooser.this.selectedInterface + " interface.");
                }
            });
            this.uriText.setText(getPreferences(0).getString(PREFS_KEY_NAME, NodeConfiguration.DEFAULT_MASTER_URI.toString()));
            this.connectionLayout = (LinearLayout) findViewById(R.id.connection_layout);
        } catch (SocketException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public void onActivityResult(int requestCode, int resultCode, Intent intent) {
        if (requestCode == 0 && resultCode == -1) {
            String scanResultFormat = intent.getStringExtra("SCAN_RESULT_FORMAT");
            Preconditions.checkState(scanResultFormat.equals("TEXT_TYPE") || scanResultFormat.equals(IntentIntegrator.QR_CODE_TYPES));
            this.uriText.setText(intent.getStringExtra("SCAN_RESULT"));
        }
    }

    public void onBackPressed() {
        moveTaskToBack(true);
    }

    public void okButtonClicked(View unused) {
        String tmpURI = this.uriText.getText().toString();
        if (!RosURIPattern.PORT.matcher(tmpURI).find()) {
            tmpURI = String.format(Locale.getDefault(), "%s:%d/", new Object[]{tmpURI, Integer.valueOf(DEFAULT_PORT)});
            this.uriText.setText(tmpURI);
        }
        final String uri = tmpURI;
        this.uriText.setEnabled(false);
        this.connectButton.setEnabled(false);
        new AsyncTask<Void, Void, Boolean>() {
            /* access modifiers changed from: protected */
            public void onPreExecute() {
                MasterChooser.this.runOnUiThread(new Runnable() {
                    public void run() {
                        MasterChooser.this.connectionLayout.setVisibility(0);
                    }
                });
            }

            /* access modifiers changed from: protected */
            public Boolean doInBackground(Void... params) {
                try {
                    new MasterClient(new URI(uri)).getUri(GraphName.of("android/master_chooser_activity"));
                    MasterChooser.this.toast("Connected!");
                    return true;
                } catch (URISyntaxException e) {
                    MasterChooser.this.toast("Invalid URI.");
                    return false;
                } catch (XmlRpcTimeoutException e2) {
                    MasterChooser.this.toast("Master unreachable!");
                    return false;
                } catch (Exception e3) {
                    String exceptionMessage = e3.getMessage();
                    if (exceptionMessage.contains(MasterChooser.CONNECTION_EXCEPTION_TEXT)) {
                        MasterChooser.this.toast("Unable to communicate with master!");
                    } else if (exceptionMessage.contains(MasterChooser.UNKNOW_HOST_TEXT)) {
                        MasterChooser.this.toast("Unable to resolve URI hostname!");
                    } else {
                        MasterChooser.this.toast("Communication error!");
                    }
                    return false;
                }
            }

            /* access modifiers changed from: protected */
            public void onPostExecute(Boolean result) {
                MasterChooser.this.runOnUiThread(new Runnable() {
                    public void run() {
                        MasterChooser.this.connectionLayout.setVisibility(8);
                    }
                });
                if (result.booleanValue()) {
                    MasterChooser.this.addRecentMasterURI(uri);
                    MasterChooser.this.setResult(-1, MasterChooser.this.createNewMasterIntent(false, true));
                    MasterChooser.this.finish();
                    return;
                }
                MasterChooser.this.connectButton.setEnabled(true);
                MasterChooser.this.uriText.setEnabled(true);
            }
        }.execute(new Void[0]);
    }

    /* access modifiers changed from: protected */
    public void toast(final String text) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(MasterChooser.this, text, 0).show();
            }
        });
    }

    public void qrCodeButtonClicked(View unused) {
        Intent intent = new Intent(BAR_CODE_SCANNER_PACKAGE_NAME);
        intent.putExtra("SCAN_MODE", "QR_CODE_MODE");
        if (!isQRCodeReaderInstalled(intent)) {
            startActivity(new Intent("android.intent.action.VIEW", Uri.parse("market://details?id=com.google.zxing.client.android")));
        } else {
            startActivityForResult(intent, 0);
        }
    }

    public void advancedCheckboxClicked(View view) {
        boolean checked = ((CheckBox) view).isChecked();
        LinearLayout advancedOptions = (LinearLayout) findViewById(R.id.advancedOptions);
        if (checked) {
            advancedOptions.setVisibility(0);
        } else {
            advancedOptions.setVisibility(8);
        }
    }

    public Intent createNewMasterIntent(boolean newMaster, boolean isPrivate) {
        Intent intent = new Intent();
        String uri = this.uriText.getText().toString();
        intent.putExtra("ROS_MASTER_CREATE_NEW", newMaster);
        intent.putExtra("ROS_MASTER_PRIVATE", isPrivate);
        intent.putExtra(EnvironmentVariables.ROS_MASTER_URI, uri);
        intent.putExtra("ROS_MASTER_NETWORK_INTERFACE", this.selectedInterface);
        return intent;
    }

    public void newMasterButtonClicked(View unused) {
        setResult(-1, createNewMasterIntent(true, false));
        finish();
    }

    public void newPrivateMasterButtonClicked(View unused) {
        setResult(-1, createNewMasterIntent(true, true));
        finish();
    }

    public void cancelButtonClicked(View unused) {
        setResult(0);
        finish();
    }

    /* access modifiers changed from: protected */
    public boolean isQRCodeReaderInstalled(Intent intent) {
        return getPackageManager().queryIntentActivities(intent, 65536).size() > 0;
    }

    /* access modifiers changed from: private */
    public void addRecentMasterURI(String uri) {
        List<String> recentURIs = getRecentMasterURIs();
        if (!recentURIs.contains(uri)) {
            recentURIs.add(0, uri);
            if (recentURIs.size() > 5) {
                recentURIs = recentURIs.subList(0, 5);
            }
        }
        SharedPreferences.Editor editor = getPreferences(0).edit();
        editor.putString(PREFS_KEY_NAME, uri);
        for (int i = 0; i < recentURIs.size(); i++) {
            editor.putString(RECENT_PREFIX_KEY_NAME + String.valueOf(i), recentURIs.get(i));
        }
        editor.putInt(RECENT_COUNT_KEY_NAME, recentURIs.size());
        editor.apply();
    }

    private List<String> getRecentMasterURIs() {
        SharedPreferences prefs = getPreferences(0);
        int numRecent = prefs.getInt(RECENT_COUNT_KEY_NAME, 0);
        List<String> recentURIs = new ArrayList<>(numRecent);
        for (int i = 0; i < numRecent; i++) {
            String uri = prefs.getString(RECENT_PREFIX_KEY_NAME + String.valueOf(i), "");
            if (!uri.isEmpty()) {
                recentURIs.add(uri);
            }
        }
        return recentURIs;
    }

    private static class RosURIPattern {
        private static final String HTTP_PROTOCOL = "(?i:http):\\/\\/";
        public static final int HTTP_PROTOCOL_LENGTH = "http://".length();
        private static final Pattern IP_ADDRESS = Pattern.compile("((25[0-5]|2[0-4][0-9]|[0-1][0-9]{2}|[1-9][0-9]|[1-9])\\.(25[0-5]|2[0-4][0-9]|[0-1][0-9]{2}|[1-9][0-9]|[1-9]|0)\\.(25[0-5]|2[0-4][0-9]|[0-1][0-9]{2}|[1-9][0-9]|[1-9]|0)\\.(25[0-5]|2[0-4][0-9]|[0-1][0-9]{2}|[1-9][0-9]|[0-9]))");
        private static final String IRI_LABEL = "[a-zA-Z0-9[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]](?:[a-zA-Z0-9[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]\\-]{0,61}[a-zA-Z0-9[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]]){0,1}";
        private static final String LABEL_CHAR = "a-zA-Z0-9[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]";
        public static final Pattern PORT = Pattern.compile(PORT_NUMBER);
        private static final String PORT_NUMBER = "\\:\\d{1,5}\\/?";
        private static final String RELAXED_DOMAIN_NAME = ("(?:(?:[a-zA-Z0-9[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]](?:[a-zA-Z0-9[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]\\-]{0,61}[a-zA-Z0-9[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]]){0,1}(?:\\.(?=\\S))?)+|" + IP_ADDRESS + ")");
        private static final String UCS_CHAR = "[ -퟿豈-﷏ﷰ-￯𐀀-🿽𠀀-𯿽𰀀-𿿽񀀀-񏿽񐀀-񟿽񠀀-񯿽񰀀-񿿽򀀀-򏿽򐀀-򟿽򠀀-򯿽򰀀-򿿽󀀀-󏿽󐀀-󟿽󡀀-󯿽&&[^ [ - ]   　]]";
        public static final Pattern URI = Pattern.compile("((?:\\b|$|^)(?:(?:(?i:http):\\/\\/)(?:" + RELAXED_DOMAIN_NAME + ")(?:" + PORT_NUMBER + ")?)" + WORD_BOUNDARY + ")");
        private static final String WORD_BOUNDARY = "(?:\\b|$|^)";

        private RosURIPattern() {
        }
    }
}
