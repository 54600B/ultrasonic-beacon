function error = trilateration_error(botx,boty,x1,y1,x2,y2,x3,y3,d1,d2,d3,q1,q2,q3)
act_d1 = single( sqrt( (x1-botx)^2 + (y1-boty)^2 ));
act_d2 = single( sqrt( (x2-botx)^2 + (y2-boty)^2 ));
act_d3 = single( sqrt( (x3-botx)^2 + (y3-boty)^2 ));

error = single( abs((act_d1-d1)^2)*q1 + abs((act_d2-d2)^2)*q2 + abs((act_d3-d3)^2)*q3 );
end

