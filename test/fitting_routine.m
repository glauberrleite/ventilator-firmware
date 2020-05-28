readings = readtable('list.csv', 'HeaderLines', 1)
dp = table2array(readings(:,3))
flow = table2array(readings(:,1))
%Certifique-se que dp não possui sinais de entrada iguais
pp = spline(dp,flow)
xx = -1:.1:1;
plot(dp, flow)