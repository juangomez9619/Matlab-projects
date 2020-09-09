function grafica2d(o1,o2)
 clf
 hold on
 axis([0 36 0 36])
 xp=[0 o1(1) o2(1)];
 yp=[0 o1(2) o2(2)];
 plot(xp,yp,'-','LineWidth',4,'Color','r');
 plot(0,0,'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot(o1(1),o1(2),'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot(o2(1),o2(2),'*r','MarkerSize',10,'LineWidth',2);
 grid on
 pause(0.005)