function [Ireturn] = calcI(Vpv, Ipv, V)
    I = [];
    for i = 1:length(Vpv)
        if abs(V - Vpv(i)) < 0.005
            I = [I; Ipv(i)];
        end
    end
    Ireturn = median(I);
end
