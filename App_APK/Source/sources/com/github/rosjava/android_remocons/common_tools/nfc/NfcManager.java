package com.github.rosjava.android_remocons.common_tools.nfc;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.nfc.FormatException;
import android.nfc.NdefMessage;
import android.nfc.NdefRecord;
import android.nfc.NfcAdapter;
import android.nfc.Tag;
import android.nfc.TagLostException;
import android.nfc.tech.IsoDep;
import android.nfc.tech.MifareClassic;
import android.nfc.tech.MifareUltralight;
import android.nfc.tech.Ndef;
import android.nfc.tech.NdefFormatable;
import android.nfc.tech.NfcA;
import android.nfc.tech.NfcB;
import android.nfc.tech.NfcF;
import android.nfc.tech.NfcV;
import android.os.Parcelable;
import android.util.Log;
import android.widget.Toast;
import com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils.ByteArrays;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.nio.charset.Charset;
import java.util.Locale;
import org.apache.commons.lang.CharEncoding;

public class NfcManager {
    private Context mContext = null;
    private String mCurrentNdefString = "";
    private IntentFilter[] mFilters;
    private NfcAdapter mNfcAdapter = null;
    private Intent mPassedIntent = null;
    private PendingIntent mPendingIntent = null;
    private String[][] mTechList;

    public NfcManager(Context context) {
        this.mContext = context;
        this.mNfcAdapter = NfcAdapter.getDefaultAdapter(this.mContext);
        Intent targetIntent = new Intent(this.mContext, this.mContext.getClass());
        targetIntent.setFlags(536870912);
        this.mPendingIntent = PendingIntent.getActivity(this.mContext, 0, targetIntent, 0);
        IntentFilter filter_1 = new IntentFilter("android.nfc.action.NDEF_DISCOVERED");
        IntentFilter filter_2 = new IntentFilter("android.nfc.action.TECH_DISCOVERED");
        IntentFilter filter_3 = new IntentFilter("android.nfc.action.TAG_DISCOVERED");
        try {
            filter_1.addDataType("*/*");
            filter_2.addDataType("*/*");
            filter_3.addDataType("*/*");
            this.mFilters = new IntentFilter[]{filter_1, filter_2, filter_3};
            this.mTechList = new String[][]{new String[]{NfcF.class.getName()}, new String[]{MifareClassic.class.getName()}, new String[]{NfcA.class.getName()}, new String[]{NfcB.class.getName()}, new String[]{NfcV.class.getName()}, new String[]{Ndef.class.getName()}, new String[]{NdefFormatable.class.getName()}, new String[]{MifareUltralight.class.getName()}, new String[]{IsoDep.class.getName()}};
        } catch (IntentFilter.MalformedMimeTypeException e) {
            throw new RuntimeException("fail", e);
        }
    }

    public boolean checkNfcStatus() {
        return this.mNfcAdapter.isEnabled();
    }

    public boolean changeNfcStatus(boolean enable) {
        if (this.mNfcAdapter == null) {
            return false;
        }
        if (enable) {
            try {
                Method setNfcEnabled = Class.forName(this.mNfcAdapter.getClass().getName()).getDeclaredMethod("enable", new Class[0]);
                setNfcEnabled.setAccessible(true);
                return ((Boolean) setNfcEnabled.invoke(this.mNfcAdapter, new Object[0])).booleanValue();
            } catch (ClassNotFoundException e) {
                e.printStackTrace();
                return false;
            } catch (NoSuchMethodException e2) {
                e2.printStackTrace();
                return false;
            } catch (IllegalArgumentException e3) {
                e3.printStackTrace();
                return false;
            } catch (IllegalAccessException e4) {
                e4.printStackTrace();
                return false;
            } catch (InvocationTargetException e5) {
                e5.printStackTrace();
                System.out.println(e5.toString());
                return false;
            }
        } else {
            try {
                Method setNfcDisabled = Class.forName(this.mNfcAdapter.getClass().getName()).getDeclaredMethod("disable", new Class[0]);
                setNfcDisabled.setAccessible(true);
                return ((Boolean) setNfcDisabled.invoke(this.mNfcAdapter, new Object[0])).booleanValue();
            } catch (ClassNotFoundException e6) {
                e6.printStackTrace();
                return false;
            } catch (NoSuchMethodException e7) {
                e7.printStackTrace();
                return false;
            } catch (IllegalArgumentException e8) {
                e8.printStackTrace();
                return false;
            } catch (IllegalAccessException e9) {
                e9.printStackTrace();
                return false;
            } catch (InvocationTargetException e10) {
                e10.printStackTrace();
                return false;
            }
        }
    }

    public boolean enableForegroundDispatch() {
        if (this.mNfcAdapter == null) {
            return false;
        }
        this.mNfcAdapter.enableForegroundDispatch((Activity) this.mContext, this.mPendingIntent, this.mFilters, this.mTechList);
        return true;
    }

    public boolean disableForegroundDispatch() {
        if (this.mNfcAdapter == null) {
            return false;
        }
        this.mNfcAdapter.disableForegroundDispatch((Activity) this.mContext);
        return true;
    }

    public boolean onNewIntent(Intent intent) {
        this.mPassedIntent = intent;
        String action = this.mPassedIntent.getAction();
        Toast.makeText(this.mContext, action, 0).show();
        if ("android.nfc.action.TAG_DISCOVERED".equals(action) || "android.nfc.action.TECH_DISCOVERED".equals(action) || "android.nfc.action.NDEF_DISCOVERED".equalsIgnoreCase(action)) {
            return true;
        }
        return false;
    }

    public String processTag() {
        if (this.mPassedIntent == null) {
            return "NFC Tag is not discovered.";
        }
        Parcelable[] rawMsgs = this.mPassedIntent.getParcelableArrayExtra("android.nfc.extra.NDEF_MESSAGES");
        if (rawMsgs == null) {
            return "NDEF Message is null";
        }
        this.mCurrentNdefString = "";
        NdefMessage[] msgs = new NdefMessage[rawMsgs.length];
        for (int i = 0; i < rawMsgs.length; i++) {
            msgs[i] = (NdefMessage) rawMsgs[i];
            this.mCurrentNdefString += ndefMessageToString(msgs[i]);
        }
        return this.mCurrentNdefString;
    }

    public String ndefMessageToString(NdefMessage message) {
        NdefRecord[] ndef_records = message.getRecords();
        String ndefString = "" + "**Num of NdefRecord : " + ndef_records.length + "\n";
        for (int i = 0; i < ndef_records.length; i++) {
            byte[] type = ndef_records[i].getType();
            byte[] id = ndef_records[i].getId();
            byte[] pl = ndef_records[i].getPayload();
            byte[] arr = ndef_records[i].toByteArray();
            ndefString = ndefString + (("**Record No. " + i + "\n") + "- TNF=" + ndef_records[i].getTnf() + "\n - TYPE=" + ByteArrays.getHexString(type, type.length) + " " + new String(type) + "\n - ID=" + ByteArrays.getHexString(id, id.length) + " " + new String(id) + "\n - PayLoad=" + ByteArrays.getHexString(pl, pl.length) + " " + new String(pl) + "\n - ByteArray=" + ByteArrays.getHexString(arr, arr.length) + " " + new String(arr) + "\n");
        }
        return ndefString;
    }

    public byte[] getPayload() {
        Parcelable[] rawMsgs;
        if (this.mPassedIntent == null || (rawMsgs = this.mPassedIntent.getParcelableArrayExtra("android.nfc.extra.NDEF_MESSAGES")) == null) {
            return null;
        }
        NdefMessage[] msgs = new NdefMessage[rawMsgs.length];
        for (int i = 0; i < rawMsgs.length; i++) {
            msgs[i] = (NdefMessage) rawMsgs[i];
        }
        NdefRecord[] records = msgs[0].getRecords();
        if (records.length > 0) {
            return records[0].getPayload();
        }
        return null;
    }

    private NdefRecord createTextRecord(String text, Locale locale, boolean encodeInUtf8) {
        byte[] langBytes = locale.getLanguage().getBytes(Charset.forName("US-ASCII"));
        return new NdefRecord(1, NdefRecord.RTD_TEXT, new byte[0], ByteArrays.concat(new byte[]{(byte) ((char) (langBytes.length + (encodeInUtf8 ? 0 : 128)))}, langBytes, text.getBytes(Charset.forName(encodeInUtf8 ? "UTF-8" : CharEncoding.UTF_16))));
    }

    public boolean writeTextNdefMessage(String payload, boolean isAAR) {
        NdefMessage msg;
        NdefRecord record = createTextRecord(payload, Locale.KOREAN, true);
        if (isAAR) {
            msg = new NdefMessage(new NdefRecord[]{record, NdefRecord.createApplicationRecord(this.mContext.getPackageName())});
        } else {
            msg = new NdefMessage(new NdefRecord[]{record});
        }
        return writeNdefMessage(msg);
    }

    public boolean writeTextNdefMessage(byte[] payload, String AAR) {
        NdefMessage msg;
        byte[] langBytes = Locale.KOREAN.getLanguage().getBytes(Charset.forName("US-ASCII"));
        Charset forName = Charset.forName("UTF-8");
        NdefRecord record = new NdefRecord(1, NdefRecord.RTD_TEXT, new byte[0], ByteArrays.concat(new byte[]{(byte) ((char) (langBytes.length + 0))}, langBytes, payload));
        if (AAR != null) {
            msg = new NdefMessage(new NdefRecord[]{record, NdefRecord.createApplicationRecord(AAR)});
        } else {
            msg = new NdefMessage(new NdefRecord[]{record});
        }
        return writeNdefMessage(msg);
    }

    public boolean writeUriNdefMessage(String payload, String AAR) {
        NdefMessage msg;
        NdefRecord record = new NdefRecord(3, NdefRecord.RTD_URI, new byte[0], payload.getBytes());
        if (AAR != null) {
            msg = new NdefMessage(new NdefRecord[]{record, NdefRecord.createApplicationRecord(AAR)});
        } else {
            msg = new NdefMessage(new NdefRecord[]{record});
        }
        return writeNdefMessage(msg);
    }

    public boolean writeMimeNdefMessage(String payload, String AAR) {
        NdefMessage msg;
        NdefRecord record = new NdefRecord(2, ("application/" + this.mContext.getPackageName()).getBytes(), new byte[0], payload.getBytes());
        if (AAR != null) {
            msg = new NdefMessage(new NdefRecord[]{record, NdefRecord.createApplicationRecord(AAR)});
        } else {
            msg = new NdefMessage(new NdefRecord[]{record});
        }
        return writeNdefMessage(msg);
    }

    public boolean writeNdefMessage(NdefMessage message) {
        if (this.mPassedIntent == null) {
            return false;
        }
        Ndef ndefTag = Ndef.get((Tag) this.mPassedIntent.getParcelableExtra("android.nfc.extra.TAG"));
        try {
            ndefTag.connect();
            try {
                ndefTag.writeNdefMessage(message);
                try {
                    ndefTag.close();
                    return true;
                } catch (IOException e) {
                    Log.e("NfcWriter", "Close tag failed. " + e.getMessage());
                    e.printStackTrace();
                    return false;
                }
            } catch (TagLostException e2) {
                Log.e("NfcWriter", "The tag left the field. " + e2.getMessage());
                e2.printStackTrace();
                return false;
            } catch (IOException e3) {
                Log.e("NfcWriter", "Message writing failure. " + e3.getMessage());
                e3.printStackTrace();
                return false;
            } catch (FormatException e4) {
                Log.e("NfcWriter", "Malformed NDEF message. " + e4.getMessage());
                e4.printStackTrace();
                return false;
            }
        } catch (IOException e5) {
            Log.e("NfcWriter", "Connect to tag failed. " + e5.getMessage());
            e5.printStackTrace();
            return false;
        }
    }
}
