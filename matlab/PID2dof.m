function [u_nom, intstate, lastdiff] = PID2dof(pos, ref, intstate_, lastdiff_, samplingtime, mode)
global b1 c1 Kp1 Ki1 Kd1
global b0 c0 Kp0 Ki0 Kd0

intstate_ = max(intstate_,-1);
intstate_ = min(intstate_,1);

if mode
    lastdiff = (c1*ref-pos)*Kd1 - samplingtime*lastdiff_;
    u_nom = Kp1*(b1*ref-pos)+Ki1*intstate_+lastdiff;
    intstate = intstate_+samplingtime*(ref-pos);
else
    lastdiff = (c0*ref-pos)*Kd0 - samplingtime*lastdiff_;
    u_nom = Kp0*(b0*ref-pos)+Ki0*intstate_+lastdiff;
    intstate = intstate_+samplingtime*(ref-pos);

end
end

