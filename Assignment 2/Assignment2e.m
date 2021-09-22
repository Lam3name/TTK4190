A = [   -0.322 	0.052 	0.028 	-1.12 	0.002;
	0	0	1	-0.001	0
	-10.6	0	-2.87	0.46	-0.65
	6.87	0	-0.04	0.32	0.02
	0	0	0	0	-7.5];

B = [	0
	0
	0
	0
	7.5];

C = [eye(4) zeros(4,1)];

D = zeros(4,1);

sys = ss(A, B, C, D);

A2b = [-2.87 -0.65; 0 -7.5];

a = [-2.87];
b = [-0.65];
c = [1];
d = [0];

s = tf('s');

tf2b = c*inv(s*1 - a)*b + d;
Va = 580;
Vg = Va;
g = 9.81;
disturbance = deg2rad(1.5);
aileron_limit = deg2rad(30);

aileron_max = deg2rad(30);
emax_phi = deg2rad(15);
omega_n_phi = sqrt(abs(b)*(aileron_max/emax_phi));
zeta_phi = 0.0707;

kp_phi = aileron_max/emax_phi*sign(b);
kd_phi = (2*zeta_phi*omega_n_phi-a)/(b);
ki_phi = -0.15;

evan_den = -1;
evan_num = b/(s*(s^2+(a+b*kd_phi)*s+b*kp_phi));
evantf = tf(evan_num,evan_den);
k = (-10:0.05:0);
y = (0:0.05:10);
[r,o] = rlocus(evantf,k);
[num,den]=ss2tf(A,B,C,D);

W_xi = 20;

omega_n_xi = 1/W_xi*omega_n_phi;
zeta_xi = 1;
kp_xi = 2*zeta_xi*omega_n_xi*Va/g;
ki_xi = omega_n_xi^2*Va/g;

figure(1);
plot(out.heading.time, out.heading.signals.values,'b');
hold on;
plot(out.heading.time, out.ref.signals.values,'r--');
figure(2)
plot(out.heading.time, out.aileron.signals.values);