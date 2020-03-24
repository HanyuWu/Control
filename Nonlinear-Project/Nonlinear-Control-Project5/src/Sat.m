function What = Sat(W,Beta)
    if W(1)> Beta
        W(1) = Beta;
    elseif W(1) < -Beta
        W(1) = -Beta;
    end
    if W(2)> Beta
        W(2) = Beta;
    elseif W(2) < -Beta
        W(2) = -Beta;
    end
    What = [W(1);W(2)];
end

