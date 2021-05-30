package com.github.rosjava.android_remocons.common_tools.zeroconf;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Checkable;
import android.widget.CheckedTextView;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import com.github.rosjava.android_remocons.common_tools.R;
import com.github.rosjava.zeroconf_jmdns_suite.jmdns.DiscoveredService;
import java.util.ArrayList;
import java.util.Iterator;

public class DiscoveryAdapter extends ArrayAdapter<DiscoveredService> {
    private final Context context;
    private ArrayList<DiscoveredService> discoveredServices;
    private int otherServicesDrawable;
    private int targetServiceDrawable;
    private String targetServiceName;

    public class CustomCheckBox extends LinearLayout implements Checkable {
        private CheckedTextView checkbox;

        public CustomCheckBox(Context context) {
            super(context);
            View view = ((LayoutInflater) context.getSystemService("layout_inflater")).inflate(R.layout.zeroconf_master_item, this, false);
            this.checkbox = (CheckedTextView) view.findViewById(R.id.service_detail);
            addView(view);
        }

        public void setChecked(boolean checked) {
            this.checkbox.setChecked(checked);
        }

        public boolean isChecked() {
            return this.checkbox.isChecked();
        }

        public void toggle() {
            setChecked(!isChecked());
        }
    }

    public DiscoveryAdapter(Context context2, ArrayList<DiscoveredService> discoveredServices2, String targetServiceName2, int targetServiceDrawable2, int otherServicesDrawable2) {
        super(context2, R.layout.zeroconf_master_item, discoveredServices2);
        this.context = context2;
        this.discoveredServices = discoveredServices2;
        this.targetServiceName = targetServiceName2;
        this.targetServiceDrawable = targetServiceDrawable2;
        this.otherServicesDrawable = otherServicesDrawable2;
    }

    public View getView(int position, View convertView, ViewGroup parent) {
        String result;
        View v = convertView;
        if (v == null) {
            v = new CustomCheckBox(getContext());
        }
        DiscoveredService discovered_service = this.discoveredServices.get(position);
        if (discovered_service != null) {
            TextView tt = (TextView) v.findViewById(R.id.service_name);
            TextView bt = (TextView) v.findViewById(R.id.service_detail);
            if (tt != null) {
                tt.setText(discovered_service.name);
            }
            if (bt != null) {
                String result2 = "";
                Iterator<String> it = discovered_service.ipv4_addresses.iterator();
                while (it.hasNext()) {
                    String ipv4_address = it.next();
                    if (result2.equals("")) {
                        result2 = result2 + ipv4_address + ":" + discovered_service.port;
                    } else {
                        result2 = result2 + "\n" + ipv4_address + ":" + discovered_service.port;
                    }
                }
                Iterator<String> it2 = discovered_service.ipv6_addresses.iterator();
                while (it2.hasNext()) {
                    String ipv6_address = it2.next();
                    if (result2.equals("")) {
                        result = result2 + ipv6_address + ":" + discovered_service.port;
                    } else {
                        result = result2 + "\n" + ipv6_address + ":" + discovered_service.port;
                    }
                }
                bt.setText(result2);
            }
            ImageView im = (ImageView) v.findViewById(R.id.icon);
            if (im != null) {
                if (discovered_service.type.indexOf("_" + this.targetServiceName + "._tcp") == -1) {
                    if (discovered_service.type.indexOf("_" + this.targetServiceName + "._udp") == -1) {
                        im.setImageDrawable(this.context.getResources().getDrawable(this.otherServicesDrawable));
                    }
                }
                im.setImageDrawable(this.context.getResources().getDrawable(this.targetServiceDrawable));
            }
        }
        return v;
    }
}
