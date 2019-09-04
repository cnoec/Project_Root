function h_track = is_it_in_track(xi,n_states,eps)
%IS_IT_IN_TRACK Summary of this function goes here
%   Detailed explanation goes here


global Par_1 Par_2 Par_3 Par_4 Par_5 Par_6 Par_7 Par_8

h_track = zeros(2*n_states,1);

for ind = 1:n_states

    state = [xi(1,ind) xi(2,ind)];      %punto da verificare
    
    if state(1) > Par_4(1)
        centre_x = mean([Par_3(1) Par_4(1)]);
        centre_y = mean([Par_3(2) Par_4(2)]);
        dist = sqrt( (centre_x-state(1))^2 + (centre_y-state(2))^2 );   % la distanza tra la traiettoria ed il centro della circ
        h_track(((ind-1)*2+1):ind*2) = [   -dist+Par_3(3)-eps;           % settori A,B
                 dist-Par_4(3)-eps];
    %      disp('A,B');

    elseif state(1) > Par_8(1) && state(1) <= Par_4(1)
        dist = state(2);            % la distanza è la coordinata y
        if state(2) > Par_4(2)      % settore C
            h_track(((ind-1)*2+1):ind*2) = [-dist+Par_5(1)-eps; dist-Par_6(1)+eps;];
    %         disp('C');
        else
            h_track(((ind-1)*2+1):ind*2) = [-dist+Par_2(1)-eps; dist-Par_1(1)+eps;]; 
    %         disp('F');      % settore F
        end

    elseif state(1) < Par_8(1)
        centre_x = mean([Par_7(1) Par_8(1)]);
        centre_y = mean([Par_7(2) Par_8(2)]);
        dist = sqrt( (centre_x-state(1))^2 + (centre_y-state(2))^2 ); 
        h_track(((ind-1)*2+1):ind*2) = [-dist+Par_7(3)-eps;           % settori D,E
                  dist-Par_8(3)-eps];
    %     disp('D,E');

    end
end

end

