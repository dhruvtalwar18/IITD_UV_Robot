package com.github.rosjava.android_remocons.common_tools.zeroconf;

import android.annotation.SuppressLint;
import android.os.AsyncTask;
import android.util.Log;
import com.github.rosjava.zeroconf_jmdns_suite.jmdns.DiscoveredService;
import com.github.rosjava.zeroconf_jmdns_suite.jmdns.ZeroconfDiscoveryHandler;
import java.util.ArrayList;
import java.util.Iterator;

public class DiscoveryHandler implements ZeroconfDiscoveryHandler {
    /* access modifiers changed from: private */
    public ArrayList<DiscoveredService> discovered_services;
    /* access modifiers changed from: private */
    public DiscoveryAdapter discovery_adapter;

    @SuppressLint({"NewApi"})
    private class ServiceAddedTask extends AsyncTask<DiscoveredService, String, Void> {
        private ServiceAddedTask() {
        }

        /* access modifiers changed from: protected */
        @SuppressLint({"NewApi"})
        public Void doInBackground(DiscoveredService... services) {
            if (services.length == 1) {
                DiscoveredService service = services[0];
                publishProgress(new String[]{"[+] Service added: " + service.name + "." + service.type + "." + service.domain + "."});
                return null;
            }
            publishProgress(new String[]{"Error - ServiceAddedTask::doInBackground received #services != 1"});
            return null;
        }
    }

    @SuppressLint({"NewApi"})
    private class ServiceResolvedTask extends AsyncTask<DiscoveredService, String, DiscoveredService> {
        private ServiceResolvedTask() {
        }

        /* access modifiers changed from: protected */
        @SuppressLint({"NewApi"})
        public DiscoveredService doInBackground(DiscoveredService... services) {
            if (services.length == 1) {
                DiscoveredService discovered_service = services[0];
                String result = ("[=] Service resolved: " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".\n") + "    Port: " + discovered_service.port;
                Iterator<String> it = discovered_service.ipv4_addresses.iterator();
                while (it.hasNext()) {
                    result = result + "\n    Address: " + it.next();
                }
                Iterator<String> it2 = discovered_service.ipv6_addresses.iterator();
                while (it2.hasNext()) {
                    result = result + "\n    Address: " + it2.next();
                }
                publishProgress(new String[]{result});
                return discovered_service;
            }
            publishProgress(new String[]{"Error - ServiceAddedTask::doInBackground received #services != 1"});
            return null;
        }

        /* access modifiers changed from: protected */
        @SuppressLint({"NewApi"})
        public void onPostExecute(DiscoveredService discovered_service) {
            if (discovered_service != null) {
                int index = 0;
                Iterator it = DiscoveryHandler.this.discovered_services.iterator();
                while (it.hasNext() && !((DiscoveredService) it.next()).name.equals(discovered_service.name)) {
                    index++;
                }
                if (index == DiscoveryHandler.this.discovered_services.size()) {
                    DiscoveryHandler.this.discovered_services.add(discovered_service);
                    DiscoveryHandler.this.discovery_adapter.notifyDataSetChanged();
                    return;
                }
                Log.i("zeroconf", "Tried to add an existing service (fix this)");
            }
        }
    }

    @SuppressLint({"NewApi"})
    private class ServiceRemovedTask extends AsyncTask<DiscoveredService, String, DiscoveredService> {
        private ServiceRemovedTask() {
        }

        /* access modifiers changed from: protected */
        @SuppressLint({"NewApi"})
        public DiscoveredService doInBackground(DiscoveredService... services) {
            if (services.length == 1) {
                DiscoveredService discovered_service = services[0];
                publishProgress(new String[]{("[-] Service removed: " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".\n") + "    Port: " + discovered_service.port});
                return discovered_service;
            }
            publishProgress(new String[]{"Error - ServiceAddedTask::doInBackground received #services != 1"});
            return null;
        }

        /* access modifiers changed from: protected */
        public void onPostExecute(DiscoveredService discovered_service) {
            if (discovered_service != null) {
                int index = 0;
                Iterator it = DiscoveryHandler.this.discovered_services.iterator();
                while (it.hasNext() && !((DiscoveredService) it.next()).name.equals(discovered_service.name)) {
                    index++;
                }
                if (index != DiscoveryHandler.this.discovered_services.size()) {
                    DiscoveryHandler.this.discovered_services.remove(index);
                    DiscoveryHandler.this.discovery_adapter.notifyDataSetChanged();
                    return;
                }
                Log.i("zeroconf", "Tried to remove a non-existant service");
            }
        }
    }

    public DiscoveryHandler(DiscoveryAdapter discovery_adapter2, ArrayList<DiscoveredService> discovered_services2) {
        this.discovery_adapter = discovery_adapter2;
        this.discovered_services = discovered_services2;
    }

    public void serviceAdded(DiscoveredService service) {
        new ServiceAddedTask().execute(new DiscoveredService[]{service});
    }

    public void serviceRemoved(DiscoveredService service) {
        new ServiceRemovedTask().execute(new DiscoveredService[]{service});
    }

    public void serviceResolved(DiscoveredService service) {
        new ServiceResolvedTask().execute(new DiscoveredService[]{service});
    }
}
