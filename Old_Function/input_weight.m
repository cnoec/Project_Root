function cost = input_weight(u,W)

R = eye(length(u))*W;

cost = u'*R*u;
    

end

