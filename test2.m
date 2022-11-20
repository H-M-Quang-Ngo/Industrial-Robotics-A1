%%
qa = rand(1,6);
qb= rand(1,6);
qc = rand(1,6);
qe = rand(1,6);
%[q , qdd] = jtraj(qa,qb,50);
[q,qd] = mstraj([qa; qb;qc;qe],0.5,[],qa,0.01,1);

t = 1:numrows(qd);
plot(t,qd);
grid on
%legend
%%
T1 = transl(0,0,1)* trotx(-pi/3);
T2 = transl(1,2,0)* troty(-pi/3);
[T,Td] = ctraj(T1,T2,50);
t = 1:50;
plot(t,Td);
grid on
legend