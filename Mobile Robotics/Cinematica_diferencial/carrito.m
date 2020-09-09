
function carro(x,y,theta)
 
 px = 0.01*[0   0    10  15   15  10  0  0];
 py = 0.01*[0 -7.5 -7.5 -2.5 2.5 7.5 7.5 0];

 rot = [cosd(theta) -sind(theta);
       sind(theta)  cosd(theta)];
   
 p_rot = rot*[px;py];

 px_rot = x + p_rot(1,:);
 py_rot = y + p_rot(2,:);


 figure(1),plot([px_rot(1) px_rot(2) px_rot(3) px_rot(4) px_rot(5) px_rot(6) px_rot(7) px_rot(8)],...
                [py_rot(1) py_rot(2) py_rot(3) py_rot(4) py_rot(5) py_rot(6) py_rot(7) py_rot(8)]);



end