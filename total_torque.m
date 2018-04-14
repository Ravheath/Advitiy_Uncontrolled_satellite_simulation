function tau=total_torque(time,state)
tau=dist_torque(time,state)+control_torque(time,state);
