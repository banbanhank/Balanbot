clear;clc;
data = load("data7.mat");
data = data.data;
phi = data(:,1)/180*pi;
thetad = data(:,2);
u = zeros(length(phi),1);
for i=1:length(phi)
   u(i)=100; 
end
plot(phi),grid
range=80:200;

%get thetadd
dt = 0.01;
for i=2:length(thetad)
    thetadd(i-1) = (thetad(i)-thetad(i-1))/dt;
end
thetadd(length(thetad)) = thetadd(end);
thetadd = thetadd';



%smoothing
%{
head = 130;
tail = 160;
A=[head*head head 1;tail*tail tail 1;(tail+1)*(tail+1) (tail+1) 1];
coe = pinv(A)*[phi(head);phi(tail);phi(tail+1)];
for i=head+1:tail-1
    phi(i)=i*i*coe(1)+i*coe(2)+coe(3);
end
%}


%get phid
dt = 0.01; 
for i=2:length(phi)
    phid(i-1) = (phi(i)-phi(i-1))/dt;
end
phid(length(phi)) = phid(end);
phid = phid';

%get phidd
for i=2:length(phid)
    phidd(i-1) = (phid(i)-phid(i-1))/dt;
end
phidd(length(phid)) = phidd(end);
phidd = phidd';

g1 = -thetadd(range);
g2 = -2*cos(phi(range)).*phidd(range)+sin(2*phi(range)).*sec(phi(range)).*phid(range).*phid(range);
g3 = -thetad(range);
g4 = phid(range);
g5 = u(range);
g6 = -thetadd(range);
g7 = -cos(phi(range)).*thetadd(range);
g8 = thetad(range)-phid(range);
g9 = sin(phi(range));
g10 = -u(range);
G1=[g1 g2 g3 g4 g5];
G2=[g6 g7 g8 g9 g10];
E = phidd(range);
a = pinv(G1'*G1)*G1'*E
b = pinv(G2'*G2)*G2'*E
