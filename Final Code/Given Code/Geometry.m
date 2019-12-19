% ABB Geometry
d1 = 0.29;
a2 = 0.270;
a3 = 0.070;
d4 = 0.302;
d6 = 0.072;

rcom1 = [0;-d1/2;0];
rcom2 = [-a2/2;0;0];
rcom3 = [-a3/2;0;0];
rcom4 = [0;-d4/2;0];
rcom5 = [0;0;0];
rcom6 = [0; -d6/2;0];

rad1 = .2/2; h1 = d1;   m1 = 8050*h1*rad1^2*pi;
rad2 = .15/2; h2 = a2;   m2 = 8050*h2*rad2^2*pi;
rad3 = .1/2; h3 = a3;  m3 = 8050*h3*rad3^2*pi;
rad4 = .1/2; h4 = d4;  m4 = 8050*h4*rad4^2*pi;
rad5 = .05/2; h5 = .125; m5 = 8050*h5*rad5^2*pi;
rad6 = .05/2; h6 = d6;  m6 = 8050*h6*rad6^2*pi;

J1 = diag(m1*[1/12*(3*rad1^2+h1^2),rad1^2/2,1/12*(3*rad1^2+h1^2)]);
J2 = diag(m2*[rad2^2/2,1/12*(3*rad2^2+h2^2),1/12*(3*rad2^2+h2^2)]);
J3 = diag(m3*[rad3^2/2,1/12*(3*rad3^2+h3^2),1/12*(3*rad3^2+h3^2)]);
J4 = diag(m4*[1/12*(3*rad4^2+h4^2),rad4^2/2,1/12*(3*rad4^2+h4^2)]);
J5 = diag(m5*[1/12*(3*rad5^2+h5^2),rad5^2/2,1/12*(3*rad5^2+h5^2)]);
J6 = diag(m6*[1/12*(3*rad6^2+h6^2),rad6^2/2,1/12*(3*rad6^2+h6^2)]);

L1 = createLink(0,d1,-pi/2,[],rcom1, m1,J1);
L2 = createLink(a2,0,0,[],rcom2, m2,J2);
L3 = createLink(a3,0,-pi/2,[],rcom3, m3,J3);
L4 = createLink(0,d4,pi/2,[],rcom4, m4,J4);
L5 = createLink(0,0,-pi/2,[],rcom5, m5,J5);
L6 = createLink(0,d6,0,[],rcom6, m6,J6);

linkList = [L1;L2;L3;L4;L5;L6];
