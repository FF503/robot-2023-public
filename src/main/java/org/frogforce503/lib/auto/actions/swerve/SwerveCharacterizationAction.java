package org.frogforce503.lib.auto.actions.swerve;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.math.PolynomialRegression;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveCharacterizationAction implements Action {

    private static final double startDelaySecs = 2.0;
    private static final double rampRateVoltsPerSec = 0.05;
  
    private final boolean forwards;
    private final boolean isDrive;
  
    private final FeedForwardCharacterizationData dataPrimary;
    private final FeedForwardCharacterizationData dataSecondary;
    private final Consumer<Double> voltageConsumerSimple;
    private final BiConsumer<Double, Double> voltageConsumerDrive;
    private final Supplier<Double> velocitySupplierPrimary;
    private final Supplier<Double> velocitySupplierSecondary;
  
    private final Timer timer = new Timer();

    public SwerveCharacterizationAction(boolean forwards,
            FeedForwardCharacterizationData data, Consumer<Double> voltageConsumer,
            Supplier<Double> velocitySupplier) {
        this.forwards = forwards;
        this.isDrive = false;
        this.dataPrimary = data;
        this.dataSecondary = null;
        this.voltageConsumerSimple = voltageConsumer;
        this.voltageConsumerDrive = null;
        this.velocitySupplierPrimary = velocitySupplier;
        this.velocitySupplierSecondary = null;
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        if (timer.get() < startDelaySecs) {
            if (isDrive) {
                voltageConsumerDrive.accept(0.0, 0.0);
            } else {
                voltageConsumerSimple.accept(0.0);
            }
            } else {
            double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec
                * (forwards ? 1 : -1);
            if (isDrive) {
                voltageConsumerDrive.accept(voltage, voltage);
            } else {
                voltageConsumerSimple.accept(voltage);
            }
            dataPrimary.add(velocitySupplierPrimary.get(), voltage);
            if (isDrive) {
                dataSecondary.add(velocitySupplierSecondary.get(), voltage);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(27.5);
    }

    @Override
    public void done() {
        if (isDrive) {
            voltageConsumerDrive.accept(0.0, 0.0);
        } else {
            voltageConsumerSimple.accept(0.0);
        }

        timer.stop();
        dataPrimary.print();

        if (isDrive) {
            dataSecondary.print();
        }
    }

    public static class FeedForwardCharacterizationData {
        private final String name;
        private final List<Double> velocityData = new LinkedList<>();
        private final List<Double> voltageData = new LinkedList<>();
    
        public FeedForwardCharacterizationData(String name) {
          this.name = name;
        }
    
        public void add(double velocity, double voltage) {
          if (Math.abs(velocity) > 1E-4) {
            velocityData.add(Math.abs(velocity));
            voltageData.add(Math.abs(voltage));
          }
        }
    
        public void print() {
          PolynomialRegression regression = new PolynomialRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              voltageData.stream().mapToDouble(Double::doubleValue).toArray(), 1);
    
          System.out.println("FF Characterization Results (" + name + "):");
          System.out
              .println("\tCount=" + Integer.toString(velocityData.size()) + "");
          System.out.println(String.format("\tR2=%.5f", regression.R2()));
          System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
          System.out.println(String.format("\tkV=%.5f", regression.beta(1)));

          SmartDashboard.putNumber("RESULTANT_KS", regression.beta(0));
          SmartDashboard.putNumber("RESULTANT_KV", regression.beta(1));
        }
      }
    
}
