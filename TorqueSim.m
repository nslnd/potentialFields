function Tor = Torque(A, AF, Avoid, Z)
		
  Num1 = [0;0;0];
		Num2 = [0;0;0];
		Num3 = [0;0;0];
		
		[1As, 2As, 3As] = fkin(A);
		[1Af, 2Af, 3Af] = fkin(AF);
		[J1,J2,J3] = JacobRPR(1As, 2As, 3As);
		Dist1=45; 
	
		range=50;
		
		Numf1 = [1Af(1,5); 1Af(2,5); 1Af(3,5)];
		Numf2 = [2Af(1,5); 2Af(2,5); 2Af(3,5)];
		Numf3 = [3Af(1,5); 3Af(2,5); 3Af(3,5)];
		
		Numm1 = [1As (1,5); 1As (2,5); 1As (3,5)];
		Numm2= [2As (1,5); 2As (2,5); 2As(3,5)];
		Numm3 = [3As (1,5); 3As (2,5); 3As (3,5)];
		Dist2 = norm(Numm2-AF);
		dist3 = norm(Numm3-AF);
	
	
		for i = 1:4
		 P1(:,i) = (Num1 -Avoid (1:4,i))/dist1(i);
		

		 P2(:,i) = (Num2 - Avoid(1:4,i))/dist2(i);
		

		 P3(:,i) = (Num3 - Avoid(1:4,i))/dist3(i);
		end
	
		for i = 1:4
		 if dist1(i) < range
		 Num1 = Numm1 + n*(1/dist1(i)-1/range)*1/(dist1(i));
		 end
		 if dist2(i) < range
		 Num2 = Numm2 + n*(1/dist2(i)-1/range)*1/(dist2(i));
		 end
	
		 end
	
	
		end
		Tor1 = transpose(A1)*Num1;
		Tor2 = transpose(A2)*Num2;
		Tor3 = transpose(A3)*Num3;
		Tor(1) = Tor(1,2) + Tor2(1,2) + Tor3(1,2) + Torr1(1,2) + 
Torr2(1,2) + Torr3(1,2);
		Tor(2) = Tor(2,1) + Tor2(2,1) + Tor3(2,1) + Torr1(2,1) + Torr2(2,1) + Torr3(2,1);
		Tor(3) = Tor(3,2) + Tor2(3,2) + Tor3(3,2) + Torr1(3,2) + Torr2(3,2)
 + Torr3(3,2);
 
 %LATEST UPDATE
%{
%% example quiver plot code
[gx, gy] = gradient (-f);
skip = 20;

figure;

xidx = 1:skip:ncols;
yidx = 1:skip:nrows;

quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 ncols 1 nrows]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);
 %}
 
 
		end