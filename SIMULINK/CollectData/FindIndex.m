function [Index] = FindIndex(No1, No2, Array)
    Index = 0;
    for i = 1:length(Array)-1
        if Array(i)==No1 && Array(i+1)==No2
            Index = i;
        end
    end
end
