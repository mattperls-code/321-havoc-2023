package org.robolancers321.subsystems.arm.commands;

import org.robolancers321.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class MoveArmSeparate extends SequentialCommandGroup{
    public MoveArmSeparate(Arm arm, double anchorSetpoint, double floatingSetpoint){
        addCommands(new MoveAnchor(arm, anchorSetpoint), new MoveFloating(arm, floatingSetpoint));
    }
}
