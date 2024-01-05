package org.frogforce503.lib.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;

public class TaskScheduler {
    private Map<String, Task> _currentlyScheduledTasks = new HashMap<>();
    private List<String> _finishedTasks = new ArrayList<>();
    
    private static TaskScheduler instance = null;
    public static TaskScheduler getInstance() {
        if (instance == null)
            instance = new TaskScheduler();
        return instance;
    }

    private void schedule(String name, Task task) {
        if (_currentlyScheduledTasks.containsKey(name) || _finishedTasks.contains(name)) // dont reschedule an existing task -- this is how we handle loops
            return;

        task.setName(name);
        _currentlyScheduledTasks.put(name, task);
    }

    public void schedule(String name, Runnable toRun, double timeToFire) {
        this.schedule(name, new Task(toRun, Timer.getFPGATimestamp(), timeToFire));
    }

    public void schedule(String name, Runnable toRun, Runnable until, double timeToFire) {
        this.schedule(name, new Task(toRun, until, Timer.getFPGATimestamp(), timeToFire));
    }

    public void allow(String name) {
        _finishedTasks.remove(name);
    }

    public void periodic() {
        try {
            for (Task task : _currentlyScheduledTasks.values())
                task.check();
        } catch (Exception e) { }
    }
    
    
    private static class Task {
        Runnable toRun;
        Runnable until;
        double startTimestamp;
        double timeToFire;

        String name;

        public Task(Runnable toRun, Runnable until, double startTimstamp, double timeToFire) {
            this.toRun = toRun;
            this.until = until;
            this.startTimestamp = startTimstamp;
            this.timeToFire = timeToFire;
        }

        public Task(Runnable toRun, double startTimstamp, double timeToFire) {
            this(toRun, null, startTimstamp, timeToFire);
        }

        public void check() {
            if (isFinished()) {
                this.toRun.run();
                TaskScheduler.getInstance()._finishedTasks.add(this.name);
                TaskScheduler.getInstance()._currentlyScheduledTasks.remove(this.name);
            } else if (this.until != null) {
                this.until.run();
            }
        }

        public void setName(String name) {
            this.name = name;
        }

        private boolean isFinished() {
            return Timer.getFPGATimestamp() - startTimestamp >= timeToFire;
        }
    }
}
