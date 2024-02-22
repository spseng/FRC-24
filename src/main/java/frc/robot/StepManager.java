package frc.robot;

import java.util.concurrent.Callable;

public class StepManager {
    private int currentStep = 0;
    private int stepCounter = 0;

    public StepManager() {
        currentStep = 0;
        stepCounter = 0;
    }

    @SafeVarargs
    public final void runSteps(Callable<Boolean>... steps) {
        stepCounter = 0;

        for (Callable step : steps) {
            try {
                runStep(step);
            } catch (Exception e) {
                e.printStackTrace();
                System.out.println("FAILED TO FINISH AUTON STEP");
            }
        }
    }

    public void runStep(Callable<Boolean> step) throws Exception {
        if (currentStep == stepCounter) {
            if(step.call()) {
                currentStep++;
            }
        }
        stepCounter++;
    }

    public void reset() {
        currentStep = 0;
        stepCounter = 0;
    }
}
