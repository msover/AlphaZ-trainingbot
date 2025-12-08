package frc.robot.subsystems;

import frc.robot.subsystems.intake.ActiveIntake;
import frc.robot.subsystems.intake.Pivot;
import frc.robot.subsystems.intake.Storage;
import frc.robot.subsystems.intake.ActiveIntake.ActiveIntakeState;
import frc.robot.subsystems.intake.Pivot.PivotState;
import frc.robot.subsystems.intake.Storage.StorageState;

public class Intake implements Subsystem {

    public enum State {
        UP,
        INTAKE,
        REVERSE
    }

    public State state = State.UP;

    final ActiveIntake activeIntake;
    final Storage storage;
    final Pivot pivot;

    public Intake() {
        activeIntake = new ActiveIntake();
        storage = new Storage();
        pivot = new Pivot();
    }

    @Override
    public void initialize() {
        activeIntake.initialize();
        storage.initialize();
        pivot.initialize();
    }

    @Override
    public void loop() {
        activeIntake.loop();
        storage.loop();
        //pivot.loop();
        updateState();
    }

    public void setState(State state) {
        this.state = state;
        switch (state) {
            case UP:
                activeIntake.setState(ActiveIntakeState.IDLE);
                storage.setState(StorageState.IDLE);
                pivot.setState(PivotState.UP);
                break;
            case INTAKE:
                activeIntake.setState(ActiveIntakeState.INTAKE);
                storage.setState(StorageState.INTAKE);
                pivot.setState(PivotState.DOWN);
                break;
            case REVERSE:
                activeIntake.setState(ActiveIntakeState.REVERSE);
                storage.setState(StorageState.REVERSE);
                pivot.setState(PivotState.DOWN);
                break;
        }
    }

    public void updateState() {
        switch (state) {
            case UP:
                break;
            case INTAKE:
                break;
            case REVERSE:
                break;
        }
    }

    public State getState() {
        return state;
    }
    
}
