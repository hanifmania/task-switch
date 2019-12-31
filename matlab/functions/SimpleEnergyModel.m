function next_E = SimpleEnergyModel(E,Kd,charge_flag,step_size)
    if charge_flag
        % Charge rate is 5times bigger than discharge one.
        Edot = 5*Kd;
    else
        Edot = -Kd;
    end
    
    next_E = E + step_size*Edot;

end
