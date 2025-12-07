package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.outtake.Lift;
import frc.robot.subsystems.outtake.Suction;
import frc.robot.subsystems.outtake.Lift.LiftState;
import frc.robot.subsystems.outtake.Suction.SuctionState;

public class Outtake implements Subsystem {

    public enum State {
        IN,
        PASSTHROUGH,
        TRANSFER,
        TRANSFERRING,
        SCORE_LOW
    }

    public State state = State.PASSTHROUGH;

    public final Lift lift;
    public final Suction suction;

    Timer timer = new Timer();

    boolean start = false;

    public Outtake() {
        lift = new Lift();
        suction = new Suction();
    }

    @Override
    public void initialize() {
        lift.initialize();
        suction.initialize();
        timer.reset();
        timer.start();
    }

    @Override
    public void loop() {
        lift.loop();
        suction.loop();
        updateState();

        // if (!start) {
        //     setState(state);
        //     start = true;
        // }

        SmartDashboard.putString("outtake state", state.toString());
    }
    
    public void setState(State state) {
        this.state = state;
        timer.reset();
        timer.start();
        switch (state) {
            case IN:
                lift.setState(LiftState.IN);
                suction.setState(SuctionState.RELEASE);
                break;
            case PASSTHROUGH:
                lift.setState(LiftState.PASSTHROUGH);
                break;
            case TRANSFER:
                lift.setState(LiftState.TRANSFER);
                suction.setState(SuctionState.RELEASE);
                break;
            case TRANSFERRING:
                lift.setState(LiftState.TRANSFER);
                suction.setState(SuctionState.SUCK);
                break;
            case SCORE_LOW:
                lift.setState(LiftState.SCORE_LOW);
                suction.setState(SuctionState.SUCK);
                break;
        }
    }

    public void updateState() {
        switch (state) {
            case IN:
                break;
            case PASSTHROUGH:
                break;
            case TRANSFER:
                if (lift.inPosition(Lift.errorThreshold)) {
                    setState(State.TRANSFERRING);
                }

                // if (timer.get() > 2) {
                //     setState(State.TRANSFERRING);
                // }
                break;
            case TRANSFERRING:
                // boolean inPosition = suction.inPosition(Suction.errorThreshold);

                // if (!inPosition) {
                //     timer.reset();
                // }

                // if (timer.get() > Suction.suckTimeout) {
                //     setState(State.PASSTHROUGH);
                // }
                if (suction.inPosition(Suction.errorThreshold)) {
                    setState(State.PASSTHROUGH);
                }
                break;
            case SCORE_LOW:
                break;
        }
    }

    public State getState() {
        return state;
    }
}
