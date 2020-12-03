function [C,P] =
		%UPDATE remember quiver plot
        Path(firstC,lastC,error,Avoid,stepSz)
		Z=10;
		config(1,:) = firstC;
		[-,-,Number3] = fkin(firstC);
		P(1,:) = T3(1:3,4);
		i = 1;

		while((norm(C(j,:) - lastC) > error) && j<4000000) 
		
		 Tor = Torque(C(j,:),endC, Avoid,Z);
		

		

		 C(j+1,:) = C(j,:) + stepSz * Tor/norm(Tor);
		 [-,-,Number3] = fkin(C(j+1,:));
		

		 P(j+1,:) = Number3(1:3,4);
		

		 j = j+ 1;
		end
		end