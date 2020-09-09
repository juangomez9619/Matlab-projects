function grafica3d(o1,o2,o3,o4,o5,o6,o7,o8,A1)
 clf
 load('linksdata')
 plot3([-17 17],[0 0],[0 0],'-k')                                %%grafica de los ejes
 hold on
 grid on
 plot3([0 0],[-17 17],[0 0],'-k')
 plot3([0 0],[0 0],[0 27],'-k')
 xlabel('X')
 ylabel('Y')
 zlabel('Z')

 xp=[0 o1(1) o2(1) o3(1) o4(1) o5(1) o6(1) o7(1) o8(1)];
 yp=[0 o1(2) o2(2) o3(2) o4(2) o5(2) o6(2) o7(2) o8(2)];
 zp=[0 o1(3) o2(3) o3(3) o4(3) o5(3) o6(3) o7(3) o8(3)];

for i=1:length(s2.V2(:,1))
s2.V2(i,:)=A1*s2.V2(i,:)';
end
 
 plot3(s1.V1(:,1)/100,s1.V1(:,2)/100,s1.V1(:,3)/100)
 plot3(s2.V2(:,1)/100,s2.V2(:,2)/100,s2.V2(:,3)/100)
 plot3(xp,yp,zp,'-','LineWidth',3,'Color','r');
 plot3(0,0,0,'o','MarkerSize',10,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o1(1),o1(2),o1(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o2(1),o2(2),o2(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o3(1),o3(2),o3(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o4(1),o4(2),o4(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o5(1),o5(2),o5(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o6(1),o6(2),o6(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o7(1),o7(2),o7(3),'o','MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','b','LineWidth',2);
 plot3(o8(1),o8(2),o8(3),'s','MarkerSize',5,'MarkerEdgeColor','r','MarkerFaceColor','g','LineWidth',2);
 grid on
 pause(0.02)
