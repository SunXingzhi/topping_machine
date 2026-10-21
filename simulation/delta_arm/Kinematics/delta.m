d1=100;
L1=100;
L2=200;
or=[0 0 0]';
E1=[d1 0 0]';

RZ120=[cos(-4*pi/3) -sin(-4*pi/3) 0
        sin(-4*pi/3) cos(-4*pi/3) 0
        0 0 1];
RZ240=[cos(4*pi/3) -sin(4*pi/3) 0
        sin(4*pi/3) cos(4*pi/3) 0
        0 0 1];    
E2=RZ120*E1;
E3=RZ240*E1;

for i=1:1:200
    circle(i,1)=80*sin(i/200*2*pi);
    circle(i,2)=80*cos(i/200*2*pi);
    circle(i,3)=-200;
end

hold on
axis([-200 200 -200 200 -300 100]);
grid on
plot3(circle(:,1),circle(:,2),circle(:,3));

while(1)
    for i=1:1:200
        plot3([or(1),E1(1)],[or(2),E1(2)],[or(3),E1(3)],'r','linewidth',10);
        hold on
        plot3(circle(:,1),circle(:,2),circle(:,3));
        axis([-200 200 -200 200 -300 100]);
        grid on
        
        
        plot3([or(1),E2(1)],[or(2),E2(2)],[or(3),E2(3)],'r','linewidth',10);
        plot3([or(1),E3(1)],[or(2),E3(2)],[or(3),E3(3)],'r','linewidth',10);
        X=circle(i,1);
        Y=circle(i,2);
        Z=circle(i,3);
        d2=20;
        d=d2-d1;
        E4=[X,Y,Z]';
        E41=[X+d2,Y,Z]';
        E42=[X-d2*sin(pi/6),Y+d2*cos(pi/6),Z]';
        E43=[X-d2*sin(pi/6),Y-d2*cos(pi/6),Z]';
        plot3([E4(1),E41(1)],[E4(2),E41(2)],[E4(3),E41(3)],'B','linewidth',10);
        plot3([E4(1),E42(1)],[E4(2),E42(2)],[E4(3),E42(3)],'B','linewidth',10);
        plot3([E4(1),E43(1)],[E4(2),E43(2)],[E4(3),E43(3)],'B','linewidth',10);
        E42D=RZ240*E42;
        E43D=RZ120*E43;
        FT1=acos((E41(1)-d1)/sqrt((E41(1)-d1).^2+E41(3).^2))-pi;
        TH1=-asin((L1.^2-L2.^2+(E41(1)-d1).^2+E41(2).^2+E41(3).^2)/(2*L1*sqrt((E41(1)-d1).^2+E41(3).^2)))-FT1;
        
        FT2=acos((E42D(1)-d1)/sqrt((E42D(1)-d1).^2+E42D(3).^2))-pi;
        TH2=-asin((L1.^2-L2.^2+(E42D(1)-d1).^2+E42D(2).^2+E42D(3).^2)/(2*L1*sqrt((E42D(1)-d1).^2+E42D(3).^2)))-FT2;
        
        FT3=acos((E43D(1)-d1)/sqrt((E43D(1)-d1).^2+E43D(3).^2))-pi;
        TH3=-asin((L1.^2-L2.^2+(E43D(1)-d1).^2+E43D(2).^2+E43D(3).^2)/(2*L1*sqrt((E43D(1)-d1).^2+E43D(3).^2)))-FT3;
        
        E51=[d1+L1*sin(TH1),0,-L1*cos(TH1)]';
        E52D=[d1+L1*sin(TH2),0,-L1*cos(TH2)]';
        E52=RZ120*E52D;
        E53D=[d1+L1*sin(TH3),0,-L1*cos(TH3)]';
        E53=RZ240*E53D;
        plot3([E51(1),E1(1)],[E51(2),E1(2)],[E51(3),E1(3)],'K','linewidth',5);
        plot3([E51(1),E41(1)],[E51(2),E41(2)],[E51(3),E41(3)],'G','linewidth',6);
        
        plot3([E52(1),E2(1)],[E52(2),E2(2)],[E52(3),E2(3)],'K','linewidth',5);
        plot3([E52(1),E42(1)],[E52(2),E42(2)],[E52(3),E42(3)],'G','linewidth',6);
        
        plot3([E53(1),E3(1)],[E53(2),E3(2)],[E53(3),E3(3)],'K','linewidth',5);
        plot3([E53(1),E43(1)],[E53(2),E43(2)],[E53(3),E43(3)],'G','linewidth',6);
        pause(0.01);
        clf
             

    end
end


