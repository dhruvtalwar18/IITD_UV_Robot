package com.github.rosjava.android_remocons.common_tools.zeroconf;

import android.content.Context;
import android.widget.ListView;
import com.github.rosjava.zeroconf_jmdns_suite.jmdns.DiscoveredService;
import com.github.rosjava.zeroconf_jmdns_suite.jmdns.Zeroconf;
import java.io.IOException;
import java.util.ArrayList;

public class MasterSearcher {
    private ArrayList<DiscoveredService> discoveredMasters = new ArrayList<>();
    private DiscoveryAdapter discoveryAdapter;
    private DiscoveryHandler discoveryHandler;
    private Logger logger;
    private Zeroconf zeroconf;

    public MasterSearcher(Context context, ListView listView, String targetServiceName, int targetServiceDrawable, int otherServicesDrawable) {
        this.discoveryAdapter = new DiscoveryAdapter(context, this.discoveredMasters, targetServiceName, targetServiceDrawable, otherServicesDrawable);
        listView.setAdapter(this.discoveryAdapter);
        listView.setItemsCanFocus(false);
        listView.setChoiceMode(2);
        this.logger = new Logger();
        this.zeroconf = new Zeroconf(this.logger);
        this.discoveryHandler = new DiscoveryHandler(this.discoveryAdapter, this.discoveredMasters);
        this.zeroconf.setDefaultDiscoveryCallback(this.discoveryHandler);
        new DiscoverySetup(context).execute(new Zeroconf[]{this.zeroconf});
    }

    public void shutdown() {
        try {
            this.zeroconf.shutdown();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
