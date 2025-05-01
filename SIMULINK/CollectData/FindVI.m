function [V, I] = FindVI(Vpv, Ipv, Index)
    if Ipv(Index)>Ipv(Index+1) && Vpv(Index)<Vpv(Index+1)
        V = Vpv(Index+1);
        I = Ipv(Index+1);
    elseif Ipv(Index)>Ipv(Index+2) && Vpv(Index)<Vpv(Index+2)
        V = Vpv(Index+2);
        I = Ipv(Index+2);
    elseif Ipv(Index)>Ipv(Index+3) && Vpv(Index)<Vpv(Index+3)
        V = Vpv(Index+3);
        I = Ipv(Index+3);
    elseif Ipv(Index)>Ipv(Index+4) && Vpv(Index)<Vpv(Index+4)
        V = Vpv(Index+4);
        I = Ipv(Index+4);
    elseif Ipv(Index)>Ipv(Index+5) && Vpv(Index)<Vpv(Index+5)
        V = Vpv(Index+5);
        I = Ipv(Index+5);
    else
        V = 0;
        I = 0;
    end
end
