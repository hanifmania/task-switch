function[Z] = updateWeight(x, y, Z, perception)
    global AgentNum
    global X Y mesh_acc

    global delta_increase delta_decrease perception_increase
    global R
    global monitor_x monitor_y
    global weightScale T
    
    Monitored = false(mesh_acc(2),mesh_acc(1),AgentNum);

    for i=1:length(x)
%         Monitored = Monitored|((X-x(i)).^2<(monitor_x^2)&(Y-y(i)).^2<(monitor_y^2));
        Monitored(:,:,i) = ((X-x(i)).^2+(Y-y(i)).^2)<(R^2);
%         if Monitored(75,100,i)
%             perception(i) = 1;
%         else
%             perception(i) = 0;
%         end
        
    end
    
%     Monitored
%     Z = Z-delta_decrease.*Monitored;
    sumMonitor = any(Monitored,3);
    Z = Z-delta_decrease*Z.*sumMonitor*T;
%     Z(sumMonitor) = 0;
    Z(Z<0.01) = 0.01;
%     for i=1:length(x)
%         Z = Z + (delta_decrease+perception_increase)*(perception(i)&Monitored(:,:,i));
%     end
    Z = Z+delta_increase*(weightScale-Z).*~sumMonitor*T;
%     Z(Z>1) = 1;
end