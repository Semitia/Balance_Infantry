function output = LQRCalc(K, x, x0)
    return K * (x0 - x);
end
