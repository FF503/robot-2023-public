package org.frogforce503.lib.util;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Shuffleboard & NetworkTables Utility/Helper Class
 */
public class snUtil {
    ArrayList<String> lNameList = new ArrayList<String>();
    ArrayList<ShuffleboardLayout> lList = new ArrayList<ShuffleboardLayout>();
    ArrayList<NetworkTable> nList = new ArrayList<NetworkTable>();

    ArrayList<String> cNameList = new ArrayList<String>();
    ArrayList<GenericEntry> cList = new ArrayList<GenericEntry>();

    ArrayList<String> scNameList = new ArrayList<String>();
    ArrayList<SendableChooser<Object>> scList = new ArrayList<SendableChooser<Object>>();

    // ---------------------- COMPONENTS ---------------------- //
    
    /**
     * Makes a new Shuffleboard component
     * 
     * With Widget & Properties
     * 
     * @param tabName Tab name of component
     * @param cName Component name
     * @param s Value of component
     * @param posX Shuffleboard X position
     * @param posY Shuffleboard Y position
     * @param width Shuffleboard layout width
     * @param height Shuffleboard layout height
     */
    public void initComponent(String tabName, String cName, Object s, int posX, int posY, int width, int height, BuiltInWidgets widget, Map<String, Object> properties) {
        GenericEntry c = Shuffleboard.getTab(tabName).add(cName, s).withPosition(posX, posY).withSize(width, height).getEntry();
        cList.add(c);
        cNameList.add(cName);
    }

    /**
     * Makes a new Shuffleboard component
     * 
     * With Widget & Without Properties
     * 
     * @param tabName Tab name of component
     * @param cName Component name
     * @param s Value of component
     * @param posX Shuffleboard X position
     * @param posY Shuffleboard Y position
     * @param width Shuffleboard layout width
     * @param height Shuffleboard layout height
     */
    public void initComponent(String tabName, String cName, Object s, int posX, int posY, int width, int height, BuiltInWidgets widget) {
        GenericEntry c = Shuffleboard.getTab(tabName).add(cName, s).withPosition(posX, posY).withSize(width, height).getEntry();
        cList.add(c);
        cNameList.add(cName);
    }
    
    /**
     * Makes a new Shuffleboard component
     * 
     * Without Widget & With Properties
     * 
     * @param tabName Tab name of component
     * @param cName Component name
     * @param s Value of component
     * @param posX Shuffleboard X position
     * @param posY Shuffleboard Y position
     * @param width Shuffleboard layout width
     * @param height Shuffleboard layout height
     */
    public void initComponent(String tabName, String cName, Object s, int posX, int posY, int width, int height, Map<String, Object> properties) {
        GenericEntry c = Shuffleboard.getTab(tabName).add(cName, s).withPosition(posX, posY).withSize(width, height).getEntry();
        cList.add(c);
        cNameList.add(cName);
    }

    /**
     * Makes a new Shuffleboard component
     * 
     * Without Widget & Properties
     * 
     * @param tabName Tab name of component
     * @param cName Component name
     * @param s Value of component
     * @param posX Shuffleboard X position
     * @param posY Shuffleboard Y position
     * @param width Shuffleboard layout width
     * @param height Shuffleboard layout height
     */
    public void initComponent(String tabName, String cName, Object s, int posX, int posY, int width, int height) {
        GenericEntry c = Shuffleboard.getTab(tabName).add(cName, s).withPosition(posX, posY).withSize(width, height).getEntry();
        cList.add(c);
        cNameList.add(cName);
    }

    /**
     * Updates component value
     * 
     * @param cName name of component
     * @param i value of component
     */
    public void updateComponent(String cName, String s) {
        cList.get(cNameList.indexOf(cName)).setString(s);
    }

    /**
     * Updates component value
     * 
     * @param cName name of component
     * @param s value of component
     */
    public void updateComponent(String cName, double d) {
        cList.get(cNameList.indexOf(cName)).setDouble(d);
    }

    /**
     * Updates component value
     * 
     * @param cName name of component
     * @param s value of component
     */
    public void updateComponent(String cName, int i) {
        cList.get(cNameList.indexOf(cName)).setInteger(i);
    }

    /**
     * Updates component value
     * 
     * @param cName name of component
     * @param s value of component
     */
    public void updateComponent(String cName, boolean b) {
        cList.get(cNameList.indexOf(cName)).setBoolean(b);
    }

    /**
     * Gets value of component
     * 
     * @param cName name of component
     * @param entryValue value of double entry
     */
    public double getComponent(String cName, double entryValue) {
        return cList.get(cNameList.indexOf(cName)).getDouble(entryValue);
    }

    /**
     * Gets value of component
     * 
     * @param cName name of component
     * @param entryValue value of double entry
     */
    public long getComponent(String cName, int entryValue) {
        return cList.get(cNameList.indexOf(cName)).getInteger(entryValue);
    }

    /**
     * Gets value of component
     * 
     * @param cName name of component
     * @param entryValue value of double entry
     */
    public String getComponent(String cName, String entryValue) {
        return cList.get(cNameList.indexOf(cName)).getString(entryValue);
    }

    /**
     * Gets value of component
     * 
     * @param cName name of component
     * @param entryValue value of double entry
     */
    public boolean getComponent(String cName, boolean entryValue) {
        return cList.get(cNameList.indexOf(cName)).getBoolean(entryValue);
    }

    // ---------------------- LAYOUTS ---------------------- //

    /**
     * Makes a new Shuffleboard layout and corresponding NetworkTable
     * 
     * @param tabName Tab Name of Layout
     * @param layoutName Layout Name
     * @param layoutType Type of Layout (in BuiltInLayouts)
     * @param posX Shuffleboard X position
     * @param posY Shuffleboard Y position
     * @param width Shuffleboard layout width
     * @param height Shuffleboard layout height
     * 
     */
    public void initLayout(String tabName, String layoutName, BuiltInLayouts layoutType, int posX, int posY, int width, int height) {
        ShuffleboardLayout l = Shuffleboard.getTab(tabName).getLayout(layoutName, layoutType).withPosition(posX, posY).withSize(width, height);
        NetworkTable n = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getSubTable(layoutName);
        lList.add(l);
        nList.add(n);
        lNameList.add(layoutName);
    }

    //Shuffleboard Adding Methods for Layouts

    /**
     * With Widget & Properties
     * 
     * @param layoutName name of Shuffleboard layout
     * @param cName name of component added
     * @param s Value of component (Use lambda expression)
     * @param cWidth width of component
     * @param cHeight height of component
     * @param widget widget of component (how it should appear on Shuffleboard)
     * @param properties properties of component
     */
    public void add(String layoutName, String cName, Object s, int cWidth, int cHeight, BuiltInWidgets widget, Map<String, Object> properties) {
        lList.get(lNameList.indexOf(layoutName)).add(cName, s).withSize(cWidth, cHeight).withWidget(widget).withProperties(properties);
    }

    /**
     * //Without Properties & With Widget
     * 
     * @param layoutName name of Shuffleboard layout
     * @param cName name of component added
     * @param s Value of component (Use lambda expression)
     * @param cWidth width of component
     * @param cHeight height of component
     * @param widget widget of component (how it should appear on Shuffleboard)
     */
    public void add(String layoutName, String cName, Object s, int cWidth, int cHeight, BuiltInWidgets widget) {
        lList.get(lNameList.indexOf(layoutName)).add(cName, s).withSize(cWidth, cHeight).withWidget(widget);
    }
    
    /**
     * Without Widget & With Properties
     * 
     * @param layoutName name of Shuffleboard layout
     * @param cName name of component added
     * @param s Value of component (Use lambda expression)
     * @param cWidth width of component
     * @param cHeight height of component
     * @param properties properties of component
     */
    public void add(String layoutName, String cName, Object s, int cWidth, int cHeight, Map<String, Object> properties) {
        lList.get(lNameList.indexOf(layoutName)).add(cName, s).withSize(cWidth, cHeight).withProperties(properties);
    }

    /**
     * Without Widget & Properties
     * 
     * @param layoutName name of Shuffleboard layout
     * @param cName name of component added
     * @param s Value of component (Use lambda expression)
     * @param cWidth width of component
     * @param cHeight height of component
     */
    public void add(String layoutName, String cName, Object s, int cWidth, int cHeight) {
        lList.get(lNameList.indexOf(layoutName)).add(cName, s).withSize(cWidth, cHeight);
    }

    //NetworkTable Methods for Layouts

    /**
     * Sets value of a NetworkTable entry
     * 
     * @param layoutName name of Shuffleboard layout corresponding to this NetworkTable
     * @param entry name of entry
     * @param entryValue value of entry
     */
    public boolean set(String layoutName, String entry, Object entryValue) {
        return nList.get(lNameList.indexOf(layoutName)).getEntry(entry).setValue(entryValue);
    }

    /**
     * Gets value of a NetworkTable entry
     * 
     * @param layoutName name of Shuffleboard layout corresponding to this NetworkTable
     * @param entry name of entry
     * @param entryValue value of double entry
     */
    public double get(String layoutName, String entry, double entryValue) {
        return nList.get(lNameList.indexOf(layoutName)).getEntry(entry).getDouble(entryValue);
    }

    /**
     * Gets value of a NetworkTable entry
     * 
     * @param layoutName name of Shuffleboard layout corresponding to this NetworkTable
     * @param entry name of entry
     * @param entryValue value of double entry
     */
    public long get(String layoutName, String entry, int entryValue) {
        return nList.get(lNameList.indexOf(layoutName)).getEntry(entry).getInteger(entryValue);
    }

    /**
     * Gets value of a NetworkTable entry
     * 
     * @param layoutName name of Shuffleboard layout corresponding to this NetworkTable
     * @param entry name of entry
     * @param entryValue value of string entry
     */
    public String get(String layoutName, String entry, String entryValue) {
        return nList.get(lNameList.indexOf(layoutName)).getEntry(entry).getString(entryValue);
    }

    /**
     * Gets value of a NetworkTable entry
     * 
     * @param layoutName name of Shuffleboard layout corresponding to this NetworkTable
     * @param entry name of entry
     * @param entryValue value of boolean entry
     */
    public boolean get(String layoutName, String entry, boolean entryValue) {
        return nList.get(lNameList.indexOf(layoutName)).getEntry(entry).getBoolean(entryValue);
    }

    // ---------------------- SendableChooser ---------------------- //

    /**
     * Makes a new SendableChooser
     * 
     * @param scName Name of SendableChooser
     */
    public void initChooser(String scName) {
        SendableChooser<Object> sc = new SendableChooser<Object>();
        scNameList.add(scName);
        scList.add(sc);
    }

    /**
     * Adds default option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param defaultValue Value of option
     */
    public void setDefaultOption(String scName, String name, int defaultValue) {
        scList.get(scNameList.indexOf(scName)).setDefaultOption(name, (Object) defaultValue);
    }

    /**
     * Adds default option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param defaultValue Value of option
     */
    public void setDefaultOption(String scName, String name, double defaultValue) {
        scList.get(scNameList.indexOf(scName)).setDefaultOption(name, (Object) defaultValue);
    }

    /**
     * Adds default option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param defaultValue Value of option
     */
    public void setDefaultOption(String scName, String name, String defaultValue) {
        scList.get(scNameList.indexOf(scName)).setDefaultOption(name, (Object) defaultValue);
    }

    /**
     * Adds default option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param defaultValue Value of option
     */
    public void setDefaultOption(String scName, String name, boolean defaultValue) {
        scList.get(scNameList.indexOf(scName)).setDefaultOption(name, (Object) defaultValue);
    }

    /**
     * Adds option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param defaultValue Value of option
     */
    public void addOption(String scName, String name, int value) {
        scList.get(scNameList.indexOf(scName)).addOption(name, (Object) value);
    }

    /**
     * Adds option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param defaultValue Value of option
     */
    public void addOption(String scName, String name, double value) {
        scList.get(scNameList.indexOf(scName)).addOption(name, (Object) value);
    }

    /**
     * Adds option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param value Value of option
     */
    public void addOption(String scName, String name, String value) {
        scList.get(scNameList.indexOf(scName)).addOption(name, (Object) value);
    }

    /**
     * Adds option to Sendable Chooser
     * 
     * @param scName Name of SendableChooser
     * @param name Name of option
     * @param value Value of option
     */
    public void addOption(String scName, String name, boolean value) {
        scList.get(scNameList.indexOf(scName)).addOption(name, (Object) value);
    }

    /**
     * Adds multiple options at a time
     * 
     * @param scName
     * @param name
     * @param value
     */
    public void addOption(String scName, String[] name, int[] value) {
        for (int i = 0; i < name.length; i++) {
            scList.get(scNameList.indexOf(scName)).addOption(name[i], (Object) value[i]);
        }
    }

    /**
     * Adds multiple options at a time
     * 
     * @param scName
     * @param name
     * @param value
     */
    public void addOption(String scName, String[] name, double[] value) {
        for (int i = 0; i < name.length; i++) {
            scList.get(scNameList.indexOf(scName)).addOption(name[i], (Object) value[i]);
        }
    }

    /**
     * Adds multiple options at a time
     * 
     * @param scName
     * @param name
     * @param value
     */
    public void addOption(String scName, String[] name, String[] value) {
        for (int i = 0; i < name.length; i++) {
            scList.get(scNameList.indexOf(scName)).addOption(name[i], (Object) value[i]);
        }
    }

    /**
     * Adds multiple options at a time
     * 
     * @param scName
     * @param name
     * @param value
     */
    public void addOption(String scName, String[] name, boolean[] value) {
        for (int i = 0; i < name.length; i++) {
            scList.get(scNameList.indexOf(scName)).addOption(name[i], (Object) value[i]);
        }
    }
}