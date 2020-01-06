function [CBF] = getChargeCBF(xi,yi,Ei,Emin,Kd,k_charge,charge_pos,radius_charge)

    pos_robot = [xi;yi];
    CBF.hx = Ei - Emin - (Kd/k_charge) * log((norm(pos_robot-charge_pos)/radius_charge));
%     CBF.hx = Ei - Emin - (Kd/k_charge) * ((norm(pos_robot-charge_pos) - radius_charge));
    CBF.dhx = -(Kd/k_charge) * ( (pos_robot-charge_pos)/norm(pos_robot-charge_pos)^2 );
%     CBF.dhx = -(Kd/k_charge) *  (pos_robot-charge_pos)/norm(pos_robot-charge_pos) ;
    CBF.Kd = Kd;
end

