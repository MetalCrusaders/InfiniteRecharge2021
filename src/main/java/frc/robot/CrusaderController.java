package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class CrusaderController extends Joystick {
    
    public CrusaderController(int port) {
        super(port);
    }

    public JoystickButton xButton = new JoystickButton(this, 3);
    public JoystickButton yButton = new JoystickButton(this, 4);
    public JoystickButton aButton = new JoystickButton(this, 1);
    public JoystickButton bButton = new JoystickButton(this, 2);
    public JoystickButton rightBumper = new JoystickButton(this, 6);
    public JoystickButton leftBumper = new JoystickButton(this, 5);
    public JoystickButton startButton = new JoystickButton(this, 8);
    public JoystickButton selectButton = new JoystickButton(this, 7);
    public JoystickButton leftStickButton = new JoystickButton(this, 9);
    public JoystickButton rightStickButton = new JoystickButton(this, 10);

    
    
    public double getLeftStickX() {
		return this.getRawAxis(0);
	}

	public double getLeftStickY() {
		return this.getRawAxis(1);
	}

	public double getRightStickX() {
		return this.getRawAxis(4);
	}

	public double getRightStickY() {
        return this.getRawAxis(5);
    }

    public double getLeftTrigger() {
        return this.getRawAxis(2);
    };

    public double getRightTrigger() {
        return this.getRawAxis(3);
    }
}
