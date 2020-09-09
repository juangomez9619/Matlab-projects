function grafica3d(o1,o2,o3)
 clf
 plot3([-31 31],[0 0],[0 0],'-k')                                %%grafica de los ejes
 hold on
 grid on
 plot3([0 0],[-31 31],[0 0],'-k')
 plot3([0 0],[0 0],[0 25],'-k')
 xlabel('X')
 ylabel('Y')
 zlabel('Z')

 xp=[0 o1(1) o2(1) o3(1)];
 yp=[0 o1(2) o2(2) o3(2)];
 zp=[0 o1(3) o2(3) o3(3)];
 plot3(xp,yp,zp,'-','LineWidth',4,'Color','r');
 plot3(0,0,0,'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o1(1),o1(2),o1(3),'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o2(1),o2(2),o2(3),'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o3(1),o3(2),o3(3),'*r','MarkerSize',10,'LineWidth',2);
 grid on
 pause(0.005)
