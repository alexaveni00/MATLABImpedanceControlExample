function T1Eq = GravityCompT1(Fdx,Fdy,d1,d2,g,l1,l2,m1,m2,th1,th2,thdot1,thdot2)
%GravityCompT1
%    T1Eq = GravityCompT1(Fdx,Fdy,D1,D2,G,L1,L2,M1,M2,TH1,TH2,THDOT1,THDOT2)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    11-Feb-2025 18:58:30

t2 = sin(th1);
t3 = sin(th2);
t4 = th1+th2;
t5 = sin(t4);
T1Eq = Fdy.*l1.*t2+Fdy.*l2.*t5+Fdx.*l2.*cos(t4)+Fdx.*l1.*cos(th1)-d1.*g.*m1.*t2-d2.*g.*m2.*t5-g.*l1.*m2.*t2-d2.*l1.*m2.*t3.*thdot2.^2-d2.*l1.*m2.*t3.*thdot1.*thdot2.*2.0;
end
